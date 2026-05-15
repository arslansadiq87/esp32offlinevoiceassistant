# ESP32-S3 Offline Voice Assistant

PlatformIO Arduino project for an offline voice assistant using:

- ESP32-S3 development board
- INMP441 I2S microphone
- 1.77 inch ST7735-compatible SPI TFT display
- 5V relay module
- Edge Impulse exported Arduino library

The assistant listens continuously. When the model detects `marvin` above 0.80 confidence, the TFT shows `Listening...` and the next 5 seconds are treated as a command window. During that window, `turn_on` turns the relay on and `turn_off` turns it off. The `noise` class is ignored.

## Wiring

### TFT Display

| TFT Pin | ESP32-S3 Pin |
| --- | --- |
| CS | GPIO 15 |
| DC | GPIO 16 |
| RST | GPIO 17 |
| MOSI | GPIO 13 |
| SCLK | GPIO 14 |
| VCC | 3.3V |
| GND | GND |
| LED / BL | 3.3V |

### INMP441 Microphone

| INMP441 Pin | ESP32-S3 Pin |
| --- | --- |
| WS | GPIO 4 |
| SCK | GPIO 5 |
| SD | GPIO 6 |
| VDD | 3.3V |
| GND | GND |
| L/R | GND |

### Relay

| Relay Pin | ESP32-S3 Pin |
| --- | --- |
| IN | GPIO 7 |
| VCC | 5V |
| GND | GND |

## Edge Impulse Training

1. Create an Edge Impulse project for audio classification.
2. Add these labels exactly: `noise`, `marvin`, `turn_on`, `turn_off`.
3. Collect 16 kHz mono audio for each class. Record `noise` from the actual room where the assistant will run.
4. Keep each command short and consistent. Record multiple speakers and distances if possible.
5. Use an Audio/MFE or MFCC processing block.
6. Train and test until live classification reliably separates the four labels.
7. Keep the labels in the exported model aligned with the constants in `src/main.cpp`.

## Export The Model

1. In Edge Impulse, go to Deployment.
2. Choose Arduino library.
3. Build and download the generated `.zip`.
4. Extract it.
5. Place the extracted library folder in this project as:

```text
lib/voice_assistant_inferencing/
```

The folder must contain:

```text
voice_assistant_inferencing.h
```

If your exported header has a different name, edit this line in `src/main.cpp`:

```cpp
#include <voice_assistant_inferencing.h>
```

If your class names are different, edit:

```cpp
LABEL_NOISE
LABEL_WAKE_WORD
LABEL_TURN_ON
LABEL_TURN_OFF
```

## Upload And Test

Build and upload:

```bash
pio run -t upload
```

Open Serial Monitor:

```bash
pio device monitor -b 115200
```

Expected behavior:

1. TFT shows `Ready`.
2. Say `marvin`.
3. If confidence is above 0.80, TFT shows `Listening...`.
4. Say `turn on` within 5 seconds to turn the relay on.
5. Say `marvin`, then `turn off` within 5 seconds to turn the relay off.
6. Serial Monitor prints every label confidence, assistant state, and relay state.

If the relay is inverted, set `RELAY_ACTIVE_HIGH` to `false` in `src/main.cpp`.
