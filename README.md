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

The test exists in two builds: an Arduino sketch (`FactoryTest.ino`) and a
UIFlow 2 MicroPython port (`factory_test.py`). The MicroPython build is the one
published to M5Burner.

## Repository layout

| Path | Purpose |
| --- | --- |
| `factory_test.py` | MicroPython build, runs under UIFlow 2. This is what ships as firmware. |
| `FactoryTest.ino` | Arduino build, for flashing from the Arduino IDE. |
| `fft.cpp`, `fft.h` | Radix-2 FFT used by the Arduino microphone test (third-party, MIT). |
| `line3D.cpp`, `line3D.h` | 3D line maths for the Arduino IMU orientation cube. |
| `tools/provision_for_export.py` | Sets a Core2 up for a firmware export: installs `main.py`, sets the boot option, verifies. |
| `tools/inspect_firmware.py` | Audits an exported `.bin` for stale builds and leaked credentials before publishing. |
| `M5BURNER_LISTING.md` | Exact field text for the M5Burner publish form. |

## Requirements

Before running this project, make sure you have the following installed in the Arduino IDE:

Libraries:
* M5Unified library
* Adafruit DMA neopixel library

Board Manager:
* M5Stack Board Manager
This requires an Additional Boards Manager URL in preferences/settings:
https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json

You will also need the following hardware:

* M5Stack Core2
* USB-C cable
* Angle Unit (Port A)
* Dual Button Unit (Port C)
* M5GO Bottom2, for the 10x SK6812 LED bar
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

Differences from the Arduino version: the microphone test shows a single input
level meter rather than the C++ build's spectrum bars, and the
vibration/RTC/microSD tests report on screen if a firmware build does not
expose the needed API instead of crashing.

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

## Publishing to M5Burner

M5Burner distributes firmware as a single `.bin`, and UIFlow 2 has no "compile
my MicroPython project to a .bin" button — the project source lives in the
cloud and a build is only ever pushed straight to a device. The supported route
is therefore:

> set one Core2 up exactly as the end user should receive it, then use
> M5Burner's **Firmware Exporter** to dump that device's entire flash to a
> `.bin`, and publish that image.

The exported image is a full 16 MB flash dump: bootloader, partition table,
the UIFlow 2 MicroPython runtime, and the filesystem holding `main.py`. That is
why the device has to be in its final state before you export.

### 1. Prepare the device

```bash
python -m pip install pyserial
python tools/provision_for_export.py --port COM5 --clean
```

Use `--list` first if you just want to see what is currently on `/flash`.
On macOS/Linux the port looks like `/dev/tty.wchusbserial*`; if both
`tty.usbmodem*` and `tty.wchusbserial*` appear, pick `tty.wchusbserial*`.

The script uploads `factory_test.py` to the device as `/flash/main.py`, hashes
it back off the device to confirm the write, removes any files UIFlow 2 did not
ship (`--clean`), and sets the UIFlow 2 boot option to `0`.

That boot option is what makes the published firmware useful: it is stored in
NVS under the `uiflow` namespace, and

| `boot_option` | behaviour on power-up |
| --- | --- |
| `0` | runs `main.py` directly |
| `1` | UIFlow 2 startup menu + network setup (factory default) |
| `2` | network setup only |

Left at `1`, whoever flashes your firmware gets the UIFlow startup screen
instead of the test. After provisioning, the boot log reads `Skip sync` rather
than `Startup with network type: WIFI`, and the test appears immediately.

UIFlow 2's boot-time override (hold a key during the first 100 ms to force the
startup menu) only covers Cardputer Adv, StickS3 and StackChan — **not the
Core2**. To get a Core2 back to the UIFlow 2 launcher, either:

```bash
python tools/provision_for_export.py --port COM5 --boot-option 1
```

(`main.py` runs in a loop but still yields to Ctrl-C over serial, which is how
the script gets a REPL), or re-burn UIFlow 2 from M5Burner. The ESP32's
bootloader lives in ROM, so the board is always recoverable over USB.

### 2. Scrub anything you do not want to publish

An export is the *whole* 16 MB flash, not just the program:

```
nvs        data  nvs       0x00009000       24576 bytes
phy_init   data  phy       0x0000f000        4096 bytes
factory    app   factory   0x00010000     7405568 bytes   <- UIFlow 2 runtime
sys        data  fat       0x00720000     1048576 bytes
vfs        data  fat       0x00820000     8253440 bytes   <- main.py lives here
```

Two things ride along that you probably do not want to hand out:

**Wi-Fi credentials.** UIFlow 2 stores them in NVS under the `uiflow`
namespace as `ssid0..2` / `pswd0..2`, along with `server` and the static-IP
settings. A dump taken from a device that has been through UIFlow's Wi-Fi setup
contains all of them.

**Deleted files.** FAT unlinks a file without erasing the blocks behind it, so
old data survives in the image. Verified on this device: after deleting the
previous `main.py` and a stray `Hola.py`, their contents were still findable in
the dump.

The tests never need Wi-Fi — the Wi-Fi test only scans for nearby networks — so
a clean public image is built from a blank flash:

1. `python -m esptool --port COM5 erase-flash`
2. Re-burn UIFlow 2 from M5Burner and **skip the Wi-Fi configuration step**
3. Re-run `tools/provision_for_export.py --port COM5`
4. Export, then audit (below)

Skip this only if you are distributing privately via a **Share** code and are
happy for the recipients to have those credentials.

### 2b. Audit the exported image

```bash
python tools/inspect_firmware.py exported.bin
```

It prints the partition table, lists which NVS keys exist (names only — it
never reads the stored values), flags the sensitive ones, and confirms the
image actually contains the current `factory_test.py`. It exits non-zero if
anything needs a look before a public upload.

### 3. Export the `.bin`

1. Open **M5Burner** and log in with your M5Stack community account.
2. **USER CUSTOM** (bottom left) → **Firmware Exporter**.
3. Select the Core2's port, choose an export path, and click **Start**.
4. Wait for the progress bar to reach 100%; the `.bin` lands in the path you
   chose.

### 4. Publish it

1. **USER CUSTOM** → **Publish**.
2. Fill in the form. [M5BURNER_LISTING.md](M5BURNER_LISTING.md) holds the exact
   text for every field — name, version, device type, Github link and the full
   description — so the listing stays consistent between releases.
3. Attach the exported `.bin` as **FirmWare**, add a **Cover** image, and click
   **Upload**.

Afterwards the entry offers **Detail** (edit the metadata), **Publish** (toggle
public visibility), **Share** (get a share code for private distribution), and
**Remove**.

### Practical notes

- Flash the exported image with M5Burner or `esptool`. Third-party loaders
  (e.g. M5Launcher) have been reported to boot the image to a black screen
  without running `main.py`.
- Re-run step 1 and re-export whenever `factory_test.py` changes; the `.bin` is
  a snapshot, not a link to this repo. `inspect_firmware.py` will tell you if an
  image is stale.
- `provision_for_export.py` talks the classic raw REPL rather than using
  `mpremote`, because UIFlow 2.4.x does not complete `mpremote` 2.x's
  raw-paste handshake (`could not enter raw repl`). It also opens the port
  without asserting DTR/RTS, which on the Core2 would reset the board.

## Notes

Most parts of the M5Core2 cannot be fully tested automatically. For example, the program can play a sound through the speaker, but the user still needs to listen and confirm that the speaker is working.

## Known Limitations

* The user must manually confirm whether some tests pass.
* The port tests require the correct external units to be connected.
* The microSD test requires a microSD card to be inserted.
* MicroPython build: the display test can repeat rather than advancing on the
  first press.
* MicroPython build: `factory_test.py` still carries the unused `_fft()` helper
  and its precomputed tables from before the microphone test moved to a level
  meter. They cost startup time and RAM on the device but are never called.

## License

Copyright (c) 2026 York University. All rights reserved.
Developed at York University by Amin Mohammadi and Charles Zeng.

`fft.cpp` / `fft.h` are third-party MIT-licensed code by Robin Scheibler and
keep their own licence. See [LICENSE](LICENSE) for the full notice.
