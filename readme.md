[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/elagil/usb-i2s-bridge/main.svg)](https://results.pre-commit.ci/latest/github/elagil/usb-i2s-bridge/main)

# USB-I2S audio bridge

This is firmware for a UAC 1.0 compliant sound card.

In this implementation, it outputs to a digital I2S amplifier - the [TAS2780](https://www.ti.com/product/TAS2780).
The software is designed to run on [STM32F401](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html) - in particular the cheaply available STM32F401RB.

The project is based upon the real-time operating system [ChibiOs](https://www.chibios.org/dokuwiki/doku.php) and inspired by [a demo project](https://forum.chibios.org/viewtopic.php?f=16&t=926&start=20) with heavy modifications.

# Hardware requirements

This firmware is built for the STM32F401(RB), but can be ported to other devices that are supported by ChibiOs and have the required peripherals.

## Clocks

The firmware assumes an HSE clock source with a frequency of 24.576 MHz. Using the provided PLL settings, the processor will run at 64 MHz with the I2S peripheral PLL at 147.456 MHz. This enables

- Error-free I2S clocks for audio sample rates (e.g. 48 kHz)
- Error-free 48 MHz clocks

Find the clock tree, generated with CubeMX, below.

![clocks](./doc/images/clocks.png "Clock tree")

## GPIO

The following IO assignments are used in this firmware, but can be changed adjusted for valid replacements:

- USB connectivity on `PA11` (`USB_DM` - data negative) and `PA12` (`USB_DP` - data positive), optionally `PA9` (`OTF_FS_VBUS` - VBUS sensing, unused).
- I2S master output on `PA4` (`I2S3_WS` - left-right clock), `PC7` (`I2S3_MCK` - master clock), `PC10` (`I2S3_CK` - bit clock), and `PC12` (`I2S3_SD` - data output).
- Connection between `PA0` (`TIM2_ETR`) and `PC7` (`I2S3_MCK`). The timer peripheral uses the I2S master clock output as a counting clock input. This allows to accurately measure the sound-card's output sample rate with regard to the USB host's start of frame (SOF) frequency. For more detail, see the description of [the feedback mechanism](#feedback-mechanism).

Additional assignments are used for functionality that is not directly related to USB audio:

- USART connectivity on `PA2` (`USART2_TX` - transmit) and `PA3` (`USART2_RX` - receive) for reporting and debugging.
- I2C control lines for the connected amplifiers.
- An analog input for a volume potentiometer (unused).

# Features

Currently, the firmware supports:
- 16 bit and 32 bit / 48 kHz audio (selectable in [the audio configuration file](./source/audio/audio_settings.h))
- Mute control, which enables hardware mute on the connected amplifiers
- Volume control, which controls the amplifier volume directly

The audio stream is never manipulated.

# Design

The following sections illustrate the functionality and design choices for the firmware package.
This section will be gradually extended.

## Feedback mechanism

The USB sound-card and the host machine that it connects to do not share the same clock domain.
Therefore, the host most likely provides audio samples at a different rate than the sound card's I2S peripheral outputs, due to component variations and imperfections.
Inevitably, if the host fills up an audio buffer via USB, and the I2S peripheral consumes data from the same buffer, the clock discrepancy will cause the buffer fill level to increase or decrease over time, depending on which of the clock domains uses a faster clock.
At some point, the buffer write (USB) and read (I2S) locations will meet, and the buffer over- or underruns. This leads to audible artifacts.

In order to mitigate this, the standard suggests a feedback mechanism, where the sound device measures its actual output sample rate, with reference to the USB start of frame (SOF) clock. In full-speed USB, the SOF period is specified to be 1 ms long.
In the implementation of this firmware, a hardware timer is clocked by the I2S master clock output. The timer clock cycles for a specified amount of SOF periods - in this case 64, which equals 64 ms. From that count, a feedback value is derived and submitted to the host.

This is a control loop, where the USB device is a mere *sensor* and the host machine is the *controller*.

For more detail, see [UAC 1.0 specification](./doc/audio10.pdf). The audio feedback mechanism shall be implemented as described in *3.7.2.2 Isochronous Synch Endpoint* (p. 32).
