# USB-I2S audio bridge

This is firmware for a USB UAC 1.0 compliant sound card that outputs to a digital I2S amplifier - the [TAS2780](https://www.ti.com/product/TAS2780).
The software is designed to run on [STM32F401](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html) - in particular the cheaply available STM32F401RB.

Based upon the real-time operating system [ChibiOs](https://www.chibios.org/dokuwiki/doku.php) and inspired by [a demo project](https://forum.chibios.org/viewtopic.php?f=16&t=926&start=20) with heavy modifications.

# Features

Currently, the firmware supports:
- 16 bit / 48 kHz audio (24 bit / 48 kHz or higher is planned)
- Mute control
- Volume control

# Functionality

To be filled. This will contain a thorough description of how the firmware operates.
