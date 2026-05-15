#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "driver/i2s.h"

static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

static constexpr int PIN_I2S_WS = 4;
static constexpr int PIN_I2S_SCK = 5;
static constexpr int PIN_I2S_SD = 6;

static constexpr int PIN_TFT_CS = 15;
static constexpr int PIN_TFT_DC = 16;
static constexpr int PIN_TFT_RST = 17;
static constexpr int PIN_TFT_MOSI = 13;
static constexpr int PIN_TFT_SCLK = 14;

static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint32_t SAMPLE_RATE = 16000;
static constexpr float MIC_GAIN = 3.0f;
static constexpr size_t MARVIN_RECORD_SECONDS = 2;
static constexpr size_t ACTION_RECORD_MS = 2500;
static constexpr size_t NOISE_RECORD_MS = 2500;
static constexpr size_t MARVIN_SAMPLE_COUNT = SAMPLE_RATE * MARVIN_RECORD_SECONDS;
static constexpr size_t ACTION_SAMPLE_COUNT = (SAMPLE_RATE * ACTION_RECORD_MS) / 1000;
static constexpr size_t NOISE_SAMPLE_COUNT = (SAMPLE_RATE * NOISE_RECORD_MS) / 1000;
static constexpr size_t I2S_READ_SAMPLES = 256;
static constexpr size_t LEVEL_READ_SAMPLES = 128;
static constexpr uint32_t LEVEL_UPDATE_INTERVAL_MS = 150;

static int32_t i2sBuffer[I2S_READ_SAMPLES];
static int16_t pcmBuffer[I2S_READ_SAMPLES];

static SPIClass tftSpi(FSPI);
static Adafruit_ST7735 tft(&tftSpi, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

static int16_t i2sSampleToPcm16(int32_t sample)
{
  // INMP441 provides a 24-bit signed sample left-aligned in the 32-bit I2S word.
  const float amplified = static_cast<float>(sample >> 16) * MIC_GAIN;

  if (amplified > 32767.0f) {
    return 32767;
  }
  if (amplified < -32768.0f) {
    return -32768;
  }

  return static_cast<int16_t>(amplified);
}

void initDisplay()
{
  tftSpi.begin(PIN_TFT_SCLK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
}

static void printFittedLine(const char *text, int16_t y, uint16_t color)
{
  const int16_t availableWidth = tft.width() - 8;
  const size_t textLength = strlen(text);
  const uint8_t textSize = (textLength * 12 <= static_cast<size_t>(availableWidth)) ? 2 : 1;

  tft.setTextSize(textSize);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(4, y);
  tft.print(text);
}

void showStatus(const char *title, const char *message)
{
  tft.fillScreen(ST77XX_BLACK);
  printFittedLine(title, 12, ST77XX_CYAN);
  printFittedLine(message, 44, ST77XX_WHITE);
}

void initMic()
{
  const i2s_config_t i2sConfig = {
      .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0,
  };

  const i2s_pin_config_t pinConfig = {
      .bck_io_num = PIN_I2S_SCK,
      .ws_io_num = PIN_I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = PIN_I2S_SD,
  };

  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2sConfig, 0, nullptr));
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pinConfig));
  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_PORT));
}

uint16_t readMicLevel()
{
  size_t bytesRead = 0;
  const size_t bytesToRead = LEVEL_READ_SAMPLES * sizeof(int32_t);
  const esp_err_t result = i2s_read(I2S_PORT, i2sBuffer, bytesToRead, &bytesRead, 0);

  if (result != ESP_OK || bytesRead == 0) {
    return 0;
  }

  const size_t samplesRead = bytesRead / sizeof(int32_t);
  uint32_t levelSum = 0;

  for (size_t i = 0; i < samplesRead; ++i) {
    const int32_t sample = i2sSampleToPcm16(i2sBuffer[i]);
    levelSum += abs(sample);
  }

  return static_cast<uint16_t>(levelSum / samplesRead);
}

void recordAndSendAudio(Stream &audioSerial, size_t totalSamples)
{
  size_t samplesSent = 0;

  ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_PORT));

  audioSerial.println("START_AUDIO");
  audioSerial.flush();

  while (samplesSent < totalSamples) {
    const size_t samplesToRead = min(I2S_READ_SAMPLES, totalSamples - samplesSent);
    const size_t bytesToRead = samplesToRead * sizeof(int32_t);
    size_t bytesRead = 0;

    esp_err_t result = i2s_read(I2S_PORT, i2sBuffer, bytesToRead, &bytesRead, portMAX_DELAY);
    if (result != ESP_OK || bytesRead == 0) {
      continue;
    }

    const size_t samplesRead = bytesRead / sizeof(int32_t);
    for (size_t i = 0; i < samplesRead; ++i) {
      pcmBuffer[i] = i2sSampleToPcm16(i2sBuffer[i]);
    }

    audioSerial.write(reinterpret_cast<const uint8_t *>(pcmBuffer), samplesRead * sizeof(int16_t));
    samplesSent += samplesRead;
  }

  audioSerial.flush();
  audioSerial.println("END_AUDIO");
  audioSerial.flush();
}

static bool handleCommandFrom(Stream &commandSerial)
{
  while (commandSerial.available() > 0) {
    const char command = static_cast<char>(commandSerial.read());

    if (command == 'M' || command == 'm') {
      showStatus("Marvin", "Recording 2.0s");
      recordAndSendAudio(commandSerial, MARVIN_SAMPLE_COUNT);
      showStatus("Audio Sent", "Ready Again");
      return true;
    }

    if (command == 'O' || command == 'o') {
      showStatus("Turn On", "Recording 2.5s");
      recordAndSendAudio(commandSerial, ACTION_SAMPLE_COUNT);
      showStatus("Audio Sent", "Ready Again");
      return true;
    }

    if (command == 'F' || command == 'f') {
      showStatus("Turn Off", "Recording 2.5s");
      recordAndSendAudio(commandSerial, ACTION_SAMPLE_COUNT);
      showStatus("Audio Sent", "Ready Again");
      return true;
    }

    if (command == 'N' || command == 'n') {
      showStatus("Noise Capture", "Recording 2.5s");
      recordAndSendAudio(commandSerial, NOISE_SAMPLE_COUNT);
      showStatus("Audio Sent", "Ready Again");
      return true;
    }
  }

  return false;
}

void handleSerialCommand()
{
  if (handleCommandFrom(Serial)) {
    return;
  }

  handleCommandFrom(Serial0);
}

static void updateMicLevelDisplay()
{
  static uint32_t lastUpdateMs = 0;
  const uint32_t now = millis();

  if (now - lastUpdateMs < LEVEL_UPDATE_INTERVAL_MS) {
    return;
  }

  lastUpdateMs = now;
  const uint16_t level = readMicLevel();
  char levelText[24];
  snprintf(levelText, sizeof(levelText), "Mic Level: %u", level);

  tft.fillRect(0, 82, tft.width(), 20, ST77XX_BLACK);
  printFittedLine(levelText, 84, ST77XX_YELLOW);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial0.begin(SERIAL_BAUD);

  initDisplay();
  showStatus("Recorder Ready", "Waiting for PC");
  initMic();
}

void loop()
{
  handleSerialCommand();
  updateMicLevelDisplay();
  delay(5);
}
