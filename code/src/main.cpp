#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <driver/i2s_std.h>

// Four slices means a 1-second audio model runs inference every 250 ms.
// Edge Impulse requires this override before including the exported library.
#ifndef EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 4
#endif

#include <ESP32-S3_Voice_Assistant_inferencing.h>

// ================================================================
// Edge Impulse model notes
// ================================================================
//
// Train the Edge Impulse project as an audio classification model with
// these labels: noise, marvin, turn_on, turn_off.
//
// Recommended workflow:
// 1. Collect 16 kHz mono audio for each class. Record "noise" from the
//    real room where this device will be used.
// 2. Use an Audio/MFE or MFCC processing block in Edge Impulse.
// 3. Train a classifier for the four labels above.
// 4. Test live classification in Edge Impulse before exporting.
// 5. Export as "Arduino library".
// The exported model currently provides these labels:
//   marvin, noise, turn_off, turn_on
// If you retrain with different labels, update LABEL_WAKE_WORD,
// LABEL_TURN_ON, LABEL_TURN_OFF, and LABEL_NOISE below.

#ifndef EI_CLASSIFIER_SLICE_SIZE
#define EI_CLASSIFIER_SLICE_SIZE (EI_CLASSIFIER_RAW_SAMPLE_COUNT / EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)
#endif

// ================================================================
// Pin definitions
// ================================================================

constexpr int TFT_CS_PIN = 15;
constexpr int TFT_DC_PIN = 16;
constexpr int TFT_RST_PIN = 17;
constexpr int TFT_MOSI_PIN = 13;
constexpr int TFT_SCLK_PIN = 14;

constexpr int MIC_WS_PIN = 4;
constexpr int MIC_SCK_PIN = 5;
constexpr int MIC_SD_PIN = 6;

constexpr int RELAY_PIN = 21;

// ================================================================
// Voice assistant settings
// ================================================================

constexpr float wakeConfidenceThreshold = 0.60f;
constexpr float wakeMinimumMicLevel = 0.08f;
constexpr float turnOnConfidenceThreshold = 0.60f;
constexpr float turnOffConfidenceThreshold = 0.60f;
constexpr float commandMinimumMicLevel = 0.06f;
constexpr float commandArmQuietLevel = 0.04f;
constexpr unsigned long listeningWindowMs = 10000;
constexpr unsigned long wakeDetectionWindowMs = 1200;
constexpr unsigned long wakeCooldownMs = 1000;
constexpr unsigned long commandStartDelayMs = 250;
constexpr unsigned long commandArmQuietMs = 300;
constexpr unsigned long commandMaxArmWaitMs = 900;
constexpr unsigned long commandDetectionWindowMs = 1500;
constexpr unsigned long commandCompleteDisplayMs = 1500;
constexpr unsigned long micIndicatorBlinkMs = 250;
constexpr uint8_t wakeRequiredDetections = 3;
constexpr uint8_t commandRequiredDetections = 1;
constexpr float micLevelNoiseFloor = 250.0f;
constexpr float micLevelFullScale = 9000.0f;
constexpr bool printAllPredictions = true;
constexpr bool serialDebugEnabled = true;
constexpr bool relayTestMode = false;
constexpr unsigned long debugStatusIntervalMs = 3000;
constexpr unsigned long relayTestIntervalMs = 2000;

constexpr const char *LABEL_NOISE = "noise";
constexpr const char *LABEL_WAKE_WORD = "marvin";
constexpr const char *LABEL_TURN_ON = "turn_on";
constexpr const char *LABEL_TURN_OFF = "turn_off";

// INMP441 sends signed 24-bit samples in a 32-bit word. This shift maps
// the microphone data into the int16 range expected by Edge Impulse audio
// examples. If your model was trained with much quieter/louder audio,
// adjust MIC_GAIN first, then retrain with the same capture chain.
constexpr int MIC_SAMPLE_SHIFT = 13;
constexpr float MIC_GAIN = 3.0f;

// ================================================================
// Globals
// ================================================================

Adafruit_ST7735 tft(&SPI, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
i2s_chan_handle_t i2sMicRx = nullptr;

int16_t inferenceBuffer[EI_CLASSIFIER_SLICE_SIZE];
int32_t i2sReadBuffer[512];

bool relayState = false;
bool micReady = false;
bool wakeWordDetected = false;
bool micIndicatorLit = false;
bool commandArmed = false;
float micInputLevel = 0.0f;
int lastMicLevelWidth = -1;
unsigned long listeningStartTime = 0;
unsigned long commandCompletedTime = 0;
unsigned long commandQuietStartTime = 0;
unsigned long lastMicIndicatorUpdateMs = 0;
unsigned long lastWakeCandidateTime = 0;
unsigned long lastListeningClosedTime = 0;
unsigned long lastCommandCandidateTime = 0;
unsigned long lastDebugStatusTime = 0;
unsigned long lastRelayTestToggleTime = 0;
uint8_t wakeCandidateHits = 0;
uint8_t commandCandidateHits = 0;
String assistantState = "BOOT";
String commandCandidateLabel = "";

void beginDebugSerial() {
  if (!serialDebugEnabled) {
    return;
  }

  Serial.begin(115200);
  Serial0.begin(115200);
  delay(1200);
}

void debugPrintln(const char *message = "") {
  if (serialDebugEnabled) {
    Serial.println(message);
    Serial0.println(message);
  }
}

template <typename... Args>
void debugPrintf(const char *format, Args... args) {
  if (serialDebugEnabled) {
    Serial.printf(format, args...);
    Serial0.printf(format, args...);
  }
}

// ================================================================
// Display functions
// ================================================================

uint16_t colorForTitle(const String &title) {
  if (title == "Turning ON") {
    return ST77XX_GREEN;
  }
  if (title == "Turning OFF") {
    return ST77XX_RED;
  }
  if (title == "Listening...") {
    return ST77XX_CYAN;
  }
  if (title == "Error") {
    return ST77XX_YELLOW;
  }
  return ST77XX_WHITE;
}

void drawCenteredText(const String &text, int16_t y, uint8_t size, uint16_t color) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;

  tft.setTextSize(size);
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);

  int16_t x = (tft.width() - w) / 2;
  if (x < 0) {
    x = 0;
  }

  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.print(text);
}

void initDisplay() {
  SPI.begin(TFT_SCLK_PIN, -1, TFT_MOSI_PIN, TFT_CS_PIN);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
}

void drawMicActivityIndicator(bool active) {
  const uint16_t inactiveColor = tft.color565(45, 45, 45);
  const uint16_t dimActiveColor = tft.color565(0, 80, 0);
  const uint16_t frameColor = active ? ST77XX_GREEN : inactiveColor;
  const uint16_t dotColor = active ? (micIndicatorLit ? ST77XX_GREEN : dimActiveColor) : inactiveColor;

  tft.fillRect(102, 102, 54, 20, ST77XX_BLACK);
  tft.drawRect(104, 104, 50, 17, frameColor);
  tft.setTextSize(1);
  tft.setTextColor(frameColor);
  tft.setCursor(109, 109);
  tft.print("MIC");
  tft.fillCircle(145, 112, 4, dotColor);
}

void drawMicLevelMeter(bool force = false) {
  if (assistantState == "ERROR") {
    return;
  }

  constexpr int meterX = 10;
  constexpr int meterY = 96;
  constexpr int meterW = 84;
  constexpr int meterH = 8;
  const int fillW = static_cast<int>(micInputLevel * (meterW - 2));

  if (!force && fillW == lastMicLevelWidth) {
    return;
  }

  lastMicLevelWidth = fillW;
  tft.fillRect(meterX, meterY, meterW, meterH, ST77XX_BLACK);
  tft.drawRect(meterX, meterY, meterW, meterH, ST77XX_WHITE);

  const uint16_t barColor = micInputLevel > 0.75f
                                ? ST77XX_RED
                                : (micInputLevel > 0.45f ? ST77XX_YELLOW : ST77XX_GREEN);
  if (fillW > 0) {
    tft.fillRect(meterX + 1, meterY + 1, fillW, meterH - 2, barColor);
  }
}

void updateMicActivityIndicator(bool force = false) {
  if (assistantState == "ERROR") {
    return;
  }

  const unsigned long now = millis();
  if (!force && now - lastMicIndicatorUpdateMs < micIndicatorBlinkMs) {
    return;
  }

  lastMicIndicatorUpdateMs = now;
  micIndicatorLit = micReady ? !micIndicatorLit : false;
  drawMicActivityIndicator(micReady);
  drawMicLevelMeter(force);
}

void showStatus(String title, String message) {
  const uint16_t color = colorForTitle(title);

  tft.fillScreen(ST77XX_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), color);
  tft.drawFastHLine(8, 34, tft.width() - 16, ST77XX_BLUE);

  drawCenteredText("Offline Voice Assistant", 10, 1, ST77XX_WHITE);
  drawCenteredText(title, 48, 2, color);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 84);
  tft.print(message);

  tft.setCursor(10, 108);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("Relay: ");
  tft.print(relayState ? "ON" : "OFF");

  lastMicLevelWidth = -1;
  drawMicLevelMeter(true);
  updateMicActivityIndicator(true);
}

void showReadyScreen() {
  assistantState = "READY";
  showStatus("Ready", "Say: marvin");
}

void showListeningScreen() {
  assistantState = "LISTENING";
  showStatus("Listening...", "Say: turn on / turn off");
}

void showTurningOnScreen() {
  assistantState = "TURNING_ON";
  commandCompletedTime = millis();
  showStatus("Turning ON", "OK");
}

void showTurningOffScreen() {
  assistantState = "TURNING_OFF";
  commandCompletedTime = millis();
  showStatus("Turning OFF", "OK");
}

void showErrorScreen(String error) {
  assistantState = "ERROR";
  showStatus("Error", error);
}

// ================================================================
// Relay functions
// ================================================================

uint8_t relayOnLevel() {
  return LOW;
}

uint8_t relayOffLevel() {
  return HIGH;
}

void writeRelay(bool on) {
  const uint8_t level = on ? relayOnLevel() : relayOffLevel();
  digitalWrite(RELAY_PIN, level);
  relayState = on;
  debugPrintf("Relay GPIO%d -> %s, output level: %s\n",
              RELAY_PIN,
              on ? "ON" : "OFF",
              level == HIGH ? "HIGH" : "LOW");
}

void initRelay() {
  const uint8_t offLevel = relayOffLevel();
  digitalWrite(RELAY_PIN, offLevel);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, offLevel);
  relayState = false;
  debugPrintf("Relay initialized OFF on GPIO%d. OFF level: %s, ON level: %s\n",
              RELAY_PIN,
              offLevel == HIGH ? "HIGH" : "LOW",
              relayOnLevel() == HIGH ? "HIGH" : "LOW");
}

void relayOn() {
  writeRelay(true);
}

void relayOff() {
  writeRelay(false);
}

void runRelayTest() {
  const unsigned long now = millis();
  if (now - lastRelayTestToggleTime < relayTestIntervalMs) {
    return;
  }

  lastRelayTestToggleTime = now;
  if (relayState) {
    relayOff();
    showStatus("Relay Test", "GPIO7 should be OFF");
  } else {
    relayOn();
    showStatus("Relay Test", "GPIO7 should be ON");
  }
}

// ================================================================
// Microphone functions
// ================================================================

bool initI2SMicrophone() {
  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 4;
  channelConfig.dma_frame_num = 512;

  esp_err_t result = i2s_new_channel(&channelConfig, nullptr, &i2sMicRx);
  if (result != ESP_OK) {
    debugPrintf("I2S channel setup failed: %d\n", result);
    micReady = false;
    return false;
  }

  i2s_std_config_t stdConfig = {};
  stdConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EI_CLASSIFIER_FREQUENCY);
  stdConfig.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  stdConfig.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  stdConfig.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  stdConfig.gpio_cfg.bclk = static_cast<gpio_num_t>(MIC_SCK_PIN);
  stdConfig.gpio_cfg.ws = static_cast<gpio_num_t>(MIC_WS_PIN);
  stdConfig.gpio_cfg.dout = I2S_GPIO_UNUSED;
  stdConfig.gpio_cfg.din = static_cast<gpio_num_t>(MIC_SD_PIN);
  stdConfig.gpio_cfg.invert_flags.mclk_inv = false;
  stdConfig.gpio_cfg.invert_flags.bclk_inv = false;
  stdConfig.gpio_cfg.invert_flags.ws_inv = false;

  result = i2s_channel_init_std_mode(i2sMicRx, &stdConfig);
  if (result != ESP_OK) {
    debugPrintf("I2S standard mode setup failed: %d\n", result);
    showErrorScreen("I2S init failed");
    micReady = false;
    return false;
  }

  result = i2s_channel_enable(i2sMicRx);
  if (result != ESP_OK) {
    debugPrintf("I2S channel enable failed: %d\n", result);
    showErrorScreen("I2S enable failed");
    micReady = false;
    return false;
  }

  micReady = true;
  debugPrintf("INMP441 ready. Sample rate: %d Hz, slice: %d samples\n",
              EI_CLASSIFIER_FREQUENCY,
              EI_CLASSIFIER_SLICE_SIZE);
  return true;
}

bool readMicrophoneSamples() {
  if (!micReady) {
    return false;
  }

  size_t samplesFilled = 0;
  static float dcOffset = 0.0f;
  uint64_t amplitudeSum = 0;

  while (samplesFilled < EI_CLASSIFIER_SLICE_SIZE) {
    const size_t remainingSamples = EI_CLASSIFIER_SLICE_SIZE - samplesFilled;
    const size_t samplesToRead = min(remainingSamples, sizeof(i2sReadBuffer) / sizeof(i2sReadBuffer[0]));
    size_t bytesRead = 0;

    esp_err_t result = i2s_channel_read(
        i2sMicRx,
        i2sReadBuffer,
        samplesToRead * sizeof(i2sReadBuffer[0]),
        &bytesRead,
        1000);

    if (result != ESP_OK || bytesRead == 0) {
      debugPrintf("I2S read failed: %d, bytes: %u\n", result, static_cast<unsigned>(bytesRead));
      return false;
    }

    const size_t readSamples = bytesRead / sizeof(i2sReadBuffer[0]);
    for (size_t i = 0; i < readSamples && samplesFilled < EI_CLASSIFIER_SLICE_SIZE; ++i) {
      const float rawSample = static_cast<float>(i2sReadBuffer[i] >> MIC_SAMPLE_SHIFT);
      dcOffset = (dcOffset * 0.995f) + (rawSample * 0.005f);
      const float centered = (rawSample - dcOffset) * MIC_GAIN;
      const int16_t sample = static_cast<int16_t>(constrain(centered, -32768.0f, 32767.0f));
      inferenceBuffer[samplesFilled++] = sample;
      amplitudeSum += abs(static_cast<int32_t>(sample));
    }
  }

  const float averageAmplitude = static_cast<float>(amplitudeSum) / static_cast<float>(samplesFilled);
  const float normalizedLevel = constrain(
      (averageAmplitude - micLevelNoiseFloor) / (micLevelFullScale - micLevelNoiseFloor),
      0.0f,
      1.0f);

  if (normalizedLevel > micInputLevel) {
    micInputLevel = normalizedLevel;
  } else {
    micInputLevel = (micInputLevel * 0.70f) + (normalizedLevel * 0.30f);
  }

  return true;
}

int microphoneInferenceCallback(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; ++i) {
    out_ptr[i] = static_cast<float>(inferenceBuffer[offset + i]);
  }
  return 0;
}

// ================================================================
// Voice assistant functions
// ================================================================

void printPredictionDebug(const ei_impulse_result_t &result) {
  debugPrintln("Predictions:");
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
    debugPrintf("  %s: %.3f\n",
                result.classification[i].label,
                result.classification[i].value);
  }
  debugPrintf("State: %s | Wake active: %s | Armed: %s | Mic: %.2f | Relay: %s\n",
              assistantState.c_str(),
              wakeWordDetected ? "yes" : "no",
              commandArmed ? "yes" : "no",
              micInputLevel,
              relayState ? "ON" : "OFF");
}

void printPeriodicDebugStatus() {
  if (!serialDebugEnabled) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastDebugStatusTime < debugStatusIntervalMs) {
    return;
  }

  lastDebugStatusTime = now;
  debugPrintf("Status: %s | Mic: %.2f | Armed: %s | Wake: %s | Relay: %s\n",
              assistantState.c_str(),
              micInputLevel,
              commandArmed ? "yes" : "no",
              wakeWordDetected ? "yes" : "no",
              relayState ? "ON" : "OFF");
}

void resetWakeCandidate() {
  wakeCandidateHits = 0;
  lastWakeCandidateTime = 0;
}

void resetCommandCandidate() {
  commandCandidateLabel = "";
  commandCandidateHits = 0;
  lastCommandCandidateTime = 0;
}

void resetCommandArming() {
  commandArmed = false;
  commandQuietStartTime = 0;
  resetCommandCandidate();
}

void updateCommandArming() {
  if (!wakeWordDetected || commandArmed) {
    return;
  }

  const unsigned long now = millis();
  if (now - listeningStartTime < commandStartDelayMs) {
    return;
  }

  if (now - listeningStartTime >= commandMaxArmWaitMs) {
    commandArmed = true;
    resetCommandCandidate();
    return;
  }

  if (micInputLevel > commandArmQuietLevel) {
    commandQuietStartTime = 0;
    return;
  }

  if (commandQuietStartTime == 0) {
    commandQuietStartTime = now;
    return;
  }

  if (now - commandQuietStartTime >= commandArmQuietMs) {
    commandArmed = true;
    resetCommandCandidate();
  }
}

bool isCommandLabel(const String &label) {
  return label == LABEL_TURN_ON || label == LABEL_TURN_OFF;
}

float thresholdForCommand(const String &label) {
  if (label == LABEL_TURN_ON) {
    return turnOnConfidenceThreshold;
  }
  if (label == LABEL_TURN_OFF) {
    return turnOffConfidenceThreshold;
  }
  return 1.0f;
}

void closeListeningAfterCommand() {
  wakeWordDetected = false;
  listeningStartTime = 0;
  lastListeningClosedTime = millis();
  resetWakeCandidate();
  resetCommandArming();
}

void enterListeningMode() {
  wakeWordDetected = true;
  listeningStartTime = millis();
  resetWakeCandidate();
  resetCommandArming();
  debugPrintln("Wake word detected. Listening for command...");
  showListeningScreen();
}

void exitListeningMode() {
  wakeWordDetected = false;
  listeningStartTime = 0;
  lastListeningClosedTime = millis();
  resetWakeCandidate();
  resetCommandArming();
  debugPrintln("Listening window closed.");
  showReadyScreen();
}

void handleWakeCandidate(float confidence) {
  if (wakeWordDetected || confidence < wakeConfidenceThreshold || micInputLevel < wakeMinimumMicLevel) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastListeningClosedTime < wakeCooldownMs) {
    return;
  }

  if (now - lastWakeCandidateTime > wakeDetectionWindowMs) {
    wakeCandidateHits = 1;
  } else if (wakeCandidateHits < wakeRequiredDetections) {
    wakeCandidateHits++;
  }

  lastWakeCandidateTime = now;
  debugPrintf("Wake candidate: %.3f | Mic level: %.2f | Hits: %u/%u\n",
              confidence,
              micInputLevel,
              wakeCandidateHits,
              wakeRequiredDetections);

  if (wakeCandidateHits >= wakeRequiredDetections) {
    enterListeningMode();
  }
}

void handleDetectedCommand(String label, float confidence) {
  if (label == LABEL_NOISE) {
    return;
  }

  if (label == LABEL_WAKE_WORD) {
    debugPrintf("Detected wake label: %s | Confidence: %.3f\n", label.c_str(), confidence);
    handleWakeCandidate(confidence);
    return;
  }

  if (!wakeWordDetected) {
    return;
  }

  if (!commandArmed) {
    return;
  }

  if (!isCommandLabel(label)) {
    return;
  }

  debugPrintf("Detected command candidate: %s | Confidence: %.3f | Mic level: %.2f\n",
              label.c_str(),
              confidence,
              micInputLevel);

  const unsigned long now = millis();
  if (now - listeningStartTime < commandStartDelayMs) {
    return;
  }

  if (micInputLevel < commandMinimumMicLevel) {
    return;
  }

  if (confidence < thresholdForCommand(label)) {
    return;
  }

  if (label != commandCandidateLabel || now - lastCommandCandidateTime > commandDetectionWindowMs) {
    commandCandidateLabel = label;
    commandCandidateHits = 1;
  } else if (commandCandidateHits < commandRequiredDetections) {
    commandCandidateHits++;
  }

  lastCommandCandidateTime = now;
  if (commandCandidateHits < commandRequiredDetections) {
    debugPrintf("Waiting for command confirmation: %s (%u/%u)\n",
                label.c_str(),
                commandCandidateHits,
                commandRequiredDetections);
    return;
  }

  if (label == LABEL_TURN_ON) {
    closeListeningAfterCommand();
    relayOn();
    showTurningOnScreen();
    return;
  }

  if (label == LABEL_TURN_OFF) {
    closeListeningAfterCommand();
    relayOff();
    showTurningOffScreen();
  }
}

void runVoiceInference() {
  if ((assistantState == "TURNING_ON" || assistantState == "TURNING_OFF") &&
      millis() - commandCompletedTime > commandCompleteDisplayMs) {
    showReadyScreen();
  }

  if (wakeWordDetected && millis() - listeningStartTime > listeningWindowMs) {
    exitListeningMode();
  }

  updateMicActivityIndicator();

  if (!readMicrophoneSamples()) {
    showErrorScreen("Mic read failed");
    delay(300);
    return;
  }

  updateMicActivityIndicator();
  drawMicLevelMeter();
  updateCommandArming();

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphoneInferenceCallback;

  ei_impulse_result_t result = {};
  EI_IMPULSE_ERROR error = run_classifier_continuous(&signal, &result, false);
  if (error != EI_IMPULSE_OK) {
    debugPrintf("Edge Impulse classifier failed: %d\n", error);
    showErrorScreen("Classifier failed");
    delay(300);
    return;
  }

  if (printAllPredictions) {
    printPredictionDebug(result);
  }

  const char *bestLabel = LABEL_NOISE;
  float bestConfidence = 0.0f;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
    if (result.classification[i].value > bestConfidence) {
      bestConfidence = result.classification[i].value;
      bestLabel = result.classification[i].label;
    }
  }

  handleDetectedCommand(String(bestLabel), bestConfidence);
}

// ================================================================
// Arduino setup/loop
// ================================================================

void setup() {
  beginDebugSerial();
  debugPrintln();
  debugPrintln("Booting ESP32-S3 offline Edge Impulse voice assistant...");

  initRelay();
  initDisplay();
  if (relayTestMode) {
    showStatus("Relay Test", "Toggling GPIO7");
  } else {
    showReadyScreen();
  }

  if (!initI2SMicrophone()) {
    showErrorScreen("Mic init failed");
    return;
  }

  run_classifier_init();
  debugPrintln("Edge Impulse continuous classifier initialized.");
}

void loop() {
  if (relayTestMode) {
    runRelayTest();
    printPeriodicDebugStatus();
    delay(20);
    return;
  }

  if (!micReady) {
    printPeriodicDebugStatus();
    delay(500);
    return;
  }

  runVoiceInference();
  printPeriodicDebugStatus();
}
