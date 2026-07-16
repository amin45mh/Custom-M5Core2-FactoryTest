# M5Core2 Sequential Hardware Test

## Overview

This project is a sequential hardware test program for the **M5Stack Core2**. It tests the built-in components of the M5Core2 one at a time, allowing the user to verify that each part of the device is functioning correctly.

The test is intended to check the following components:

* PSRAM
* Port A, C
* LEDs
* Touchscreen
* Buttons A, B, and C
* IMU
* Microphone
* Speaker
* Vibration motor
* RTC
* Battery
* microSD card
* Wi-Fi

PSRAM, Wi-Fi can be tested automatically. Others require the user to observe the result and confirm that the component is working as expected.

## Requirements

Before running this project, make sure you have the following installed in the Arduino IDE:

* M5Unified library
* WiFi library
* SD library

You will also need the following hardware:

* M5Stack Core2
* USB-C cable
* Sonic sensor unit
* Angle unit
* Synth unit
* microSD card

## Setup

Connect the external modules to the M5Core2 as follows:

* Plug the **Angle Unit** into **Port A**
* Plug the **Dual Button Unit** into **Port C**

These modules are used to test whether each Grove port is functioning correctly.

## Installation

1. Install the required libraries in the Arduino IDE:

   * M5Unified
   * Adafruit NeoPixel
   * WiFi
   * SD

2. Create a sketch folder with the same name as the `.ino` file.

   For example, since the default file is named:

```text
FactoryTest.ino
```

Then the folder should be named:

```text
FactoryTest/
```

3. Place the `.ino` file inside the sketch folder.

4. Place any additional project files in the same folder directory.

5. Open the `.ino` file in Arduino IDE.

6. Connect the M5Core2 to your computer using a USB-C cable.

7. Select the correct board and port in Arduino IDE.

8. Upload the program to the M5Core2.

## Running under UIFlow 2 (MicroPython, no re-flashing)

If your Core2 already runs the UIFlow 2 firmware, you can run the same test
sequence without burning the Arduino build. Use `factory_test.py`:

1. Open the UIFlow 2 web IDE (flow.m5stack.com), switch the project to
   **Python** mode, paste the contents of `factory_test.py`, and press
   **Run** (runs once) or **Download** (persists on the device).
2. Alternatively upload it with Thonny or `mpremote` as `main.py`.

Differences from the Arduino version: the microphone visualizer uses a
smaller software FFT (slower refresh), and the vibration/RTC/microSD tests
report on screen if a firmware build does not expose the needed API instead
of crashing.

## How to Run

After the program is uploaded, the M5Core2 will begin testing each component in sequence.

Each test will display instructions or feedback on the screen. Follow the instructions for each step and verify that the current component is working properly.

## How to Use

The program moves through the hardware tests one at a time.

To move to the next test, either:

* Wait for the automatic test to finish
* Press the on-screen **PASS** or **FAIL** button, or
* Press **Button B** or **Button C** on the M5Core2 during the touch screen test

During each test, observe the screen, connected module, sound, vibration, or sensor output to determine whether the component is working correctly.

For the port tests:

* During the **Port A** test, verify that the Angle unit is responding by turning the knob.
* During the **Port C** test, verify that the Dual button unit is responding by pressing the buttons.

## Tested Components

### PSRAM

The PSRAM is tested by temporarily allocating a large chunk of memory.

### Ports

The three external ports are tested using connected M5Stack units:

* Port A: GPIO
* Port C: GPIO

### LEDs

The LED test checks whether the onboard LEDs are functioning properly.

### Touchscreen

The touchscreen test checks whether the display can detect touch input.

### Buttons

Buttons A, B, and C are tested to confirm that each button can detect input.

### IMU

The IMU test checks whether motion and orientation data can be read from the built-in inertial measurement unit.

### Microphone

The microphone test checks whether the M5Core2 can detect audio input.

### Speaker

The speaker test plays sound so the user can verify that the speaker is working.

### Vibration Motor

The vibration motor test activates the motor so the user can confirm that vibration is working.

### RTC

The RTC test checks whether the real-time clock is functioning.

### Battery

The battery test displays battery-related information, such as charge level or power status.

### microSD

The microSD test checks whether the device can detect and access a microSD card.

### Wi-Fi

The Wi-Fi test checks whether the M5Core2 can detect nearby Wifi Networks.

## Notes

Most parts of the M5Core2 cannot be fully tested automatically. For example, the program can play a sound through the speaker, but the user still needs to listen and confirm that the speaker is working.

## Known Limitations

* The user must manually confirm whether some tests pass.
* The port tests require the correct external units to be connected.
* The microSD test requires a microSD card to be inserted.
