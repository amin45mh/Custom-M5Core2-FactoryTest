# M5Core2 Sequential Factory Test - UIFlow 2 MicroPython port of FactoryTest.ino
#
# Run it on a Core2 that already has the UIFlow 2 firmware:
#   - UIFlow2 web IDE (flow.m5stack.com) in Python mode: paste and Run, or
#   - Thonny / mpremote: upload as main.py (runs on every boot) or run it once.
#
# External hardware expected (same as the Arduino version):
#   - Angle Unit on Port A (G33, analog)
#   - Dual Button Unit on Port C (G13 blue, G14 red)
#   - M5GO Bottom2 for the 10x SK6812 LED bar on G25
#
# Differences from the Arduino version:
#   - The mic visualizer uses a 256-point pure-Python FFT (no esp-dsp here),
#     so it refreshes slower than the C++ original but shows the same bars.
#   - PSRAM is tested by allocating/patterning a 100 KB buffer on the
#     MicroPython heap (which lives in PSRAM on this firmware).
#   - Vibration/RTC/SD calls are wrapped defensively: if a firmware build
#     lacks one of those bindings the test reports it on screen instead of
#     crashing, and you can still mark PASS/FAIL.

import M5
import math
import time
import gc
import os
import machine
import network
from machine import Pin, ADC

Lcd = M5.Lcd

# ---------------- colors ----------------
WHITE = 0xFFFFFF
BLACK = 0x000000
RED = 0xFF0000
GREEN = 0x00FF00
BLUE = 0x0000FF
ORANGE = 0xFF9C00      # 0xff,0x9c,0x00 from the original
FFT_BG = 0x332000      # 0x33,0x20,0x00
FFT_GREEN = 0x66FF00   # 0x66,0xff,0x00

# ---------------- test sequence ----------------
(TEST_PSRAM, TEST_PORTS, TEST_LEDS, TEST_DISPLAY, TEST_TOUCH, TEST_BUTTONS,
 TEST_IMU, TEST_MICROPHONE, TEST_SPEAKER, TEST_VIBRATION, TEST_RTC,
 TEST_BATTERY, TEST_MICROSD, TEST_WIFI, TEST_COUNT) = range(15)

TEST_NAMES = ["PSRAM", "PORTS", "LED", "DISPLAY", "TOUCH", "BUTTONS", "IMU",
              "MICROPHONE", "SPEAKER", "VIBRATION", "RTC", "BATTERY",
              "MICROSD", "WIFI"]

results = [False] * TEST_COUNT
current_test = TEST_PSRAM

# ---------------- shared touch state ----------------
_touch_prev = False
touch_pressed = False
touch_just_pressed = False
touch_x = 0
touch_y = 0


def poll_touch():
    global _touch_prev, touch_pressed, touch_just_pressed, touch_x, touch_y
    pressed = M5.Touch.getCount() > 0
    if pressed:
        touch_x = M5.Touch.getX()
        touch_y = M5.Touch.getY()
    touch_just_pressed = pressed and not _touch_prev
    _touch_prev = pressed
    touch_pressed = pressed


# ---------------- drawing helpers ----------------
def text(x, y, s, color=WHITE, size=2, bg=BLACK):
    Lcd.setTextSize(size)
    Lcd.setTextColor(color, bg)
    Lcd.setCursor(x, y)
    Lcd.print(s)


def draw_header(title):
    Lcd.fillScreen(BLACK)
    text(10, 10, title, WHITE, 2)
    if current_test == TEST_TOUCH:
        text(10, 210, "Tap BtnB to PASS or BtnC to FAIL.", WHITE, 1)


def draw_next_buttons():
    if current_test != TEST_TOUCH:
        Lcd.fillRoundRect(140, 190, 80, 40, 8, GREEN)
        text(158, 202, "PASS", WHITE, 2, GREEN)
        Lcd.fillRoundRect(230, 190, 80, 40, 8, RED)
        text(248, 202, "FAIL", WHITE, 2, RED)


def update_results(passed):
    results[current_test] = passed


def next_pressed():
    if current_test != TEST_TOUCH:
        if touch_just_pressed:
            if 140 <= touch_x <= 220 and 190 <= touch_y <= 230:  # PASS
                update_results(True)
                return True
            if 230 <= touch_x <= 310 and 190 <= touch_y <= 230:  # FAIL
                update_results(False)
                return True
    else:
        if M5.BtnB.wasPressed():
            update_results(True)
            return True
        if M5.BtnC.wasPressed():
            update_results(False)
            return True
    return False


# ---------------- LED bar (M5GO Bottom2, G25, 10 LEDs) ----------------
try:
    from hardware import RGB
    rgb = RGB()  # built-in bar: pin 25, 10 LEDs
    rgb.set_brightness(20)  # ~= strip.setBrightness(50) in the original
except Exception:
    rgb = None

# Brightness 50/255 from strip.setBrightness(50) -> ~20%.
LED_COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
led_counter = 0
led_color_counter = 0


def strip_off():
    if rgb:
        rgb.fill_color(0x000000)


# ---------------- PSRAM test ----------------
def check_psram():
    Lcd.setTextSize(2)
    gc.collect()
    try:
        buf = bytearray(100 * 1024)
    except MemoryError:
        text(20, 50, "PSRAM malloc failed", RED)
        return False
    text(20, 50, "PSRAM malloc Successful", GREEN)
    time.sleep_ms(100)

    pattern = b"\xa5" * 1024
    ok = True
    for off in range(0, len(buf), 1024):
        buf[off:off + 1024] = pattern
    for off in range(0, len(buf), 1024):
        if buf[off:off + 1024] != pattern:
            ok = False
            break
    del buf
    gc.collect()
    if ok:
        text(20, 80, "PSRAM W&R Successful", GREEN)
    else:
        text(20, 80, "PSRAM read failed", RED)
    return ok


# ---------------- IMU 3D cube ----------------
DEG = math.pi / 180
SIN_A = math.sin(19.47 * DEG)
COS_A = math.cos(19.47 * DEG)
SIN_G = math.sin(20.7 * DEG)
COS_G = math.cos(20.7 * DEG)
CUBE_CX = 160
CUBE_CY = 120

theta = 0.0
phi = 0.0
last_theta = 0.0
last_phi = 0.0
IMU_ALPHA = 0.2

# Axis line end points (start is always the origin). These persist between
# frames like the C++ globals so unrotated components keep their old values.
axis_x_end = [0.0, 0.0, 0.0]
axis_y_end = [0.0, 0.0, 0.0]
axis_z_end = [0.0, 0.0, 30.0]

_CUBE = ((-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
         (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1))
_EDGES = ((0, 1), (1, 2), (2, 3), (3, 0),
          (0, 4), (1, 5), (2, 6), (3, 7),
          (4, 5), (5, 6), (6, 7), (7, 4))
rect_source = [([a * 30.0 for a in _CUBE[s]], [a * 30.0 for a in _CUBE[e]])
               for s, e in _EDGES]
rect_dis_start = [0.0, 0.0, 0.0]
rect_dis_end = [0.0, 0.0, 0.0]

prev_cube_lines = []


def rotate_inplace(p, ax, ay, az):
    # Faithful to line3D::RotatePoint including its sequential updates
    # (each axis uses the already-updated coordinates).
    if ax:
        c = math.cos(ax * DEG)
        s = math.sin(ax * DEG)
        p[1] = p[1] * c - p[2] * s
        p[2] = p[1] * s + p[2] * c
    if ay:
        c = math.cos(ay * DEG)
        s = math.sin(ay * DEG)
        p[0] = p[2] * s + p[0] * c
        p[2] = p[2] * c - p[0] * s
    if az:
        c = math.cos(az * DEG)
        s = math.sin(az * DEG)
        p[0] = p[0] * c - p[1] * s
        p[1] = p[0] * s + p[1] * c


def rotate_to(src, dst, ax, ay, az):
    # Faithful to the two-point overload: components whose rotation is zero
    # keep whatever value dst already had.
    if ax:
        c = math.cos(ax * DEG)
        s = math.sin(ax * DEG)
        dst[1] = src[1] * c - src[2] * s
        dst[2] = src[1] * s + src[2] * c
    if ay:
        c = math.cos(ay * DEG)
        s = math.sin(ay * DEG)
        dst[0] = src[2] * s + src[0] * c
        dst[2] = src[2] * c - src[0] * s
    if az:
        c = math.cos(az * DEG)
        s = math.sin(az * DEG)
        dst[0] = src[0] * c - src[1] * s
        dst[1] = src[0] * s + src[1] * c


def project(p):
    px = p[0] * COS_G - p[1] * SIN_G
    py = -(p[0] * SIN_G * SIN_A) - (p[1] * COS_G * SIN_A) + p[2] * COS_A
    return int(px) + CUBE_CX, CUBE_CY - int(py)


def imu_cube_frame():
    global theta, phi, last_theta, last_phi, prev_cube_lines
    acc_x, acc_y, acc_z = M5.Imu.getAccel()
    if -1 < acc_x < 1:
        theta = math.asin(-acc_x) * 57.295
    if acc_z != 0:
        phi = math.atan(acc_y / acc_z) * 57.295

    theta = IMU_ALPHA * theta + (1 - IMU_ALPHA) * last_theta
    phi = IMU_ALPHA * phi + (1 - IMU_ALPHA) * last_phi

    axis_z_end[0] = 0.0
    axis_z_end[1] = 0.0
    axis_z_end[2] = 24.0
    rotate_inplace(axis_z_end, theta, phi, 0)
    rotate_to(axis_z_end, axis_x_end, -90, 0, 0)
    rotate_to(axis_z_end, axis_y_end, 0, 90, 0)

    origin = project((0, 0, 0))
    lines = [
        (origin, project(axis_x_end), GREEN),
        (origin, project(axis_y_end), GREEN),
        (origin, project(axis_z_end), GREEN),
    ]
    for src_start, src_end in rect_source:
        rotate_to(src_start, rect_dis_start, theta, phi, 0)
        rotate_to(src_end, rect_dis_end, theta, phi, 0)
        lines.append((project(rect_dis_start), project(rect_dis_end), ORANGE))

    last_theta = theta
    last_phi = phi

    # Erase last frame, then draw the new one (no canvas needed).
    for a, b, _ in prev_cube_lines:
        Lcd.drawLine(a[0], a[1], b[0], b[1], BLACK)
    for a, b, color in lines:
        Lcd.drawLine(a[0], a[1], b[0], b[1], color)
    prev_cube_lines = lines


# ---------------- microphone FFT visualizer ----------------
FFT_N = 256
MIC_RATE = 16000
FFT_X0 = 30
FFT_Y0 = 50

_fft_rev = []
for _i in range(FFT_N):
    _r = 0
    _n = _i
    for _ in range(8):  # log2(256)
        _r = (_r << 1) | (_n & 1)
        _n >>= 1
    _fft_rev.append(_r)
_fft_cos = [math.cos(-2 * math.pi * k / FFT_N) for k in range(FFT_N // 2)]
_fft_sin = [math.sin(-2 * math.pi * k / FFT_N) for k in range(FFT_N // 2)]

import array
mic_buf = array.array("h", bytearray(2 * FFT_N))
fft_prev_cols = [-1] * 24


def _fft(re, im):
    n = FFT_N
    rev = _fft_rev
    for i in range(n):
        j = rev[i]
        if j > i:
            re[i], re[j] = re[j], re[i]
            im[i], im[j] = im[j], im[i]
    size = 2
    while size <= n:
        half = size >> 1
        step = n // size
        for i in range(0, n, size):
            k = 0
            for j in range(i, i + half):
                wr = _fft_cos[k]
                wi = _fft_sin[k]
                l = j + half
                tr = wr * re[l] - wi * im[l]
                ti = wr * im[l] + wi * re[l]
                re[l] = re[j] - tr
                im[l] = im[j] - ti
                re[j] += tr
                im[j] += ti
                k += step
        size <<= 1


def mic_fft_bands():
    try:
        if not M5.Mic.record(mic_buf, MIC_RATE):
            return None
        while M5.Mic.isRecording():
            time.sleep_ms(1)
    except Exception:
        return None

    # Same +-2000 input scale as the original, x4 to keep the sensitivity of
    # its 1024-point FFT with our 256 samples.
    scale = (4000.0 / 65536.0) * 4.0
    re = [mic_buf[i] * scale for i in range(FFT_N)]
    im = [0.0] * FFT_N
    _fft(re, im)

    bands = []
    for b in range(24):
        total = 0.0
        for k in range(5):
            idx = 1 + b * 5 + k
            mag = math.sqrt(re[idx] * re[idx] + im[idx] * im[idx])
            if mag > 2000:
                mag = 2000
            total += mag
        avg = total / 5
        bands.append(int(avg * 8 // 2000))  # 0..8 like the original
    return bands


def draw_fft_column(col, value):
    sx = FFT_X0 + col * 12
    for y in range(9):
        if y < value:
            color = ORANGE
        elif y == value:
            color = FFT_GREEN
        else:
            color = FFT_BG
        Lcd.fillRect(sx, FFT_Y0 + 120 - y * 12 - 5, 10, 10, color)


def microphone_frame():
    bands = mic_fft_bands()
    if bands is None:
        return
    for col in range(24):
        if bands[col] != fft_prev_cols[col]:
            draw_fft_column(col, bands[col])
            fft_prev_cols[col] = bands[col]


# ---------------- speaker / mic switching ----------------
def mic_on():
    try:
        M5.Speaker.end()
    except Exception:
        pass
    try:
        M5.Mic.begin()
    except Exception:
        pass


def speaker_on():
    try:
        M5.Mic.end()
    except Exception:
        pass
    try:
        M5.Speaker.begin()
        M5.Speaker.setVolume(128)
    except Exception:
        pass


# ---------------- RTC / battery / SD / WiFi helpers ----------------
def rtc_datetime():
    try:
        from hardware import RTC
        return RTC().datetime()
    except Exception:
        return machine.RTC().datetime()


def sd_check():
    # 1) some UIFlow2 builds have it mounted already
    try:
        os.listdir("/sd")
        return True, "microSD detected"
    except Exception:
        pass
    # 2) UIFlow2 hardware.sdcard helper
    try:
        from hardware import sdcard
        sdcard.SDCard(slot=2, width=1, sck=18, miso=38, mosi=23, cs=4,
                      freq=20000000)
        os.listdir("/sd")
        return True, "microSD detected"
    except Exception:
        pass
    # 3) plain machine.SDCard on the shared SPI bus
    try:
        sd = machine.SDCard(slot=2, width=1, sck=18, miso=38, mosi=23, cs=4,
                            freq=20000000)
        os.mount(sd, "/sd")
        os.listdir("/sd")
        os.umount("/sd")
        try:
            sd.deinit()
        except Exception:
            pass
        return True, "microSD detected"
    except Exception as e:
        return False, "No microSD detected"


def set_vibration(level):
    try:
        M5.Power.setVibration(level)
        return True
    except Exception:
        return False


# ---------------- port test peripherals ----------------
adc_pot = None
pin_blue = None
pin_red = None
potentiometer_value = 0
blue_button_state = 0
red_button_state = 0


def setup_ports():
    global adc_pot, pin_blue, pin_red
    try:
        adc_pot = ADC(Pin(33))
        try:
            adc_pot.atten(ADC.ATTN_11DB)
        except Exception:
            pass
    except Exception:
        adc_pot = None
    pin_blue = Pin(13, Pin.IN)
    pin_red = Pin(14, Pin.IN)


# ---------------- per-test screens ----------------
start_time = 0


def setup_test_screen():
    global current_test, fft_prev_cols, prev_cube_lines

    if current_test == TEST_PORTS:
        draw_header("Port Test")
        text(10, 60, "Angle Unit: 0")
        text(10, 120, "Blue Button Pressed: No")
        text(10, 150, "Red Button Pressed: No")
        draw_next_buttons()

    elif current_test == TEST_LEDS:
        draw_header("LED Test")
        draw_next_buttons()

    elif current_test == TEST_DISPLAY:
        draw_header("Display Test")
        Lcd.fillRect(10, 50, 60, 60, RED)
        Lcd.fillRect(80, 50, 60, 60, GREEN)
        Lcd.fillRect(150, 50, 60, 60, BLUE)
        Lcd.fillRect(220, 50, 60, 60, WHITE)
        text(10, 120, "Press any of the colors.")
        draw_next_buttons()

    elif current_test == TEST_TOUCH:
        draw_header("Touch Test")
        text(10, 60, "Touch the screen.")
        draw_next_buttons()

    elif current_test == TEST_BUTTONS:
        draw_header("Button Test")
        text(10, 60, "Press BtnA, BtnB, or BtnC")
        text(10, 100, "Press BtnA", RED)
        text(10, 125, "Press BtnB", RED)
        text(10, 150, "Press BtnC", RED)
        draw_next_buttons()

    elif current_test == TEST_IMU:
        prev_cube_lines = []
        draw_header("IMU Test")
        draw_next_buttons()

    elif current_test == TEST_MICROPHONE:
        draw_header("Microphone Test")
        mic_on()
        fft_prev_cols = [-1] * 24
        Lcd.fillRect(FFT_X0, FFT_Y0, 288, 120, FFT_BG)
        draw_next_buttons()

    elif current_test == TEST_SPEAKER:
        draw_header("Speaker Test")
        text(10, 60, "Playing tone...")
        speaker_on()
        try:
            M5.Speaker.tone(1000, 500)
        except Exception:
            pass
        time.sleep_ms(600)
        text(10, 120, "Pass if you heard a beep.")
        draw_next_buttons()

    elif current_test == TEST_VIBRATION:
        draw_header("Vibration Test")
        text(10, 60, "Vibrating...")
        supported = set_vibration(200)
        time.sleep_ms(700)
        set_vibration(0)
        if supported:
            text(10, 120, "Pass if you felt a")
            text(10, 150, "vibration.")
        else:
            text(10, 120, "setVibration() missing in", RED)
            text(10, 150, "this firmware build.", RED)
        draw_next_buttons()

    elif current_test == TEST_RTC:
        draw_header("RTC Test")
        try:
            dt = rtc_datetime()
            text(10, 60, "%04d-%02d-%02d" % (dt[0], dt[1], dt[2]))
            text(10, 80, "%02d:%02d:%02d" % (dt[4], dt[5], dt[6]))
        except Exception:
            text(10, 60, "RTC read failed", RED)
        text(10, 120, "Pass if you see the")
        text(10, 140, "correct time.")
        draw_next_buttons()

    elif current_test == TEST_BATTERY:
        draw_header("Battery / Power Test")
        level = -1
        try:
            level = M5.Power.getBatteryLevel()
        except Exception:
            pass
        if level >= 0:
            update_results(True)
            charging = False
            try:
                charging = bool(M5.Power.isCharging())
            except Exception:
                pass
            text(10, 60, "Battery: %d%%" % level, GREEN)
            text(10, 80, "Charging: %s" % ("Yes" if charging else "No"),
                 GREEN)
        else:
            update_results(False)
            text(10, 120, "Power chip did not respond.", RED)
        time.sleep_ms(3000)
        next_test()

    elif current_test == TEST_MICROSD:
        draw_header("microSD Test")
        ok, msg = sd_check()
        if ok:
            text(10, 60, msg)
        else:
            text(10, 60, msg, RED)
            text(10, 80, "Pass if unused.")
        draw_next_buttons()

    elif current_test == TEST_WIFI:
        draw_header("Wi-Fi Test")
        text(10, 60, "Scanning...")
        try:
            wlan = network.WLAN(network.STA_IF)
            wlan.active(True)
            networks = wlan.scan()
        except Exception:
            networks = None
        Lcd.fillRect(10, 60, 300, 20, BLACK)
        if networks is not None:
            update_results(True)
            text(10, 60, "%d networks found:" % len(networks), GREEN)
            for i, net in enumerate(networks[:5]):
                try:
                    ssid = net[0].decode()
                except Exception:
                    ssid = str(net[0])
                text(10, 80 + 20 * i, " " + ssid)
        else:
            update_results(False)
            text(10, 60, "Wi-Fi scan failed", RED)
        try:
            wlan.active(False)
        except Exception:
            pass
        time.sleep_ms(3000)
        next_test()

    else:  # summary
        draw_header("Tests Complete:")
        for i in range(TEST_COUNT):
            color = GREEN if results[i] else RED
            text(10, 40 + i * 10, TEST_NAMES[i], color, 1)
        text(10, 220, "Reset device to run again")


def next_test():
    global current_test
    if current_test < TEST_COUNT:
        current_test += 1
        if current_test == TEST_DISPLAY:
            strip_off()
        if current_test != TEST_PORTS:
            setup_test_screen()


# ---------------- per-test loop handlers ----------------
def loop_ports():
    global potentiometer_value, blue_button_state, red_button_state
    pot = adc_pot.read() if adc_pot else 0
    blue = pin_blue.value()
    red = pin_red.value()

    if abs(pot - potentiometer_value) > 70:  # debounce for potentiometer
        potentiometer_value = pot
        Lcd.fillRect(10, 60, 240, 20, BLACK)
        text(10, 60, "Angle Unit: %d" % potentiometer_value)

    if not blue and not blue_button_state:
        blue_button_state = 1
        Lcd.fillRect(10, 120, 310, 20, BLACK)
        text(10, 120, "Blue Button Pressed: Yes")
    elif blue and blue_button_state:
        blue_button_state = 0
        Lcd.fillRect(10, 120, 310, 20, BLACK)
        text(10, 120, "Blue Button Pressed: No ")

    if not red and not red_button_state:
        red_button_state = 1
        Lcd.fillRect(10, 150, 310, 20, BLACK)
        text(10, 150, "Red Button Pressed: Yes")
    elif red and red_button_state:
        red_button_state = 0
        Lcd.fillRect(10, 150, 310, 20, BLACK)
        text(10, 150, "Red Button Pressed: No ")


def loop_leds():
    global start_time, led_counter, led_color_counter
    if rgb is None:
        return
    if time.ticks_diff(time.ticks_ms(), start_time) >= 1000:
        start_time = time.ticks_ms()
        if led_counter == 5:
            led_counter = 0
            led_color_counter += 1
        if led_color_counter == 3:
            led_color_counter = 0
        if led_counter > 0:
            rgb.set_color(led_counter - 1, 0x000000)
            rgb.set_color(led_counter + 4, 0x000000)
        else:
            rgb.set_color(4, 0x000000)
            rgb.set_color(9, 0x000000)
        color = LED_COLORS[led_color_counter]
        rgb.set_color(led_counter, color)
        rgb.set_color(led_counter + 5, color)
        led_counter += 1


def loop_display():
    if not touch_pressed:
        return
    color_pressed = False
    if 10 <= touch_x <= 70 and 50 <= touch_y <= 110:
        Lcd.fillScreen(RED)
        color_pressed = True
    elif 80 <= touch_x <= 140 and 50 <= touch_y <= 110:
        Lcd.fillScreen(GREEN)
        color_pressed = True
    elif 150 <= touch_x <= 210 and 50 <= touch_y <= 110:
        Lcd.fillScreen(BLUE)
        color_pressed = True
    elif 220 <= touch_x <= 280 and 50 <= touch_y <= 110:
        Lcd.fillScreen(WHITE)
        color_pressed = True
    if color_pressed:
        time.sleep_ms(2000)
        setup_test_screen()


def loop_touch():
    if touch_pressed:
        Lcd.fillRect(10, 100, 210, 60, BLACK)
        text(10, 100, "X: %d" % touch_x, GREEN)
        text(10, 130, "Y: %d" % touch_y, GREEN)


def loop_buttons():
    if M5.BtnA.wasPressed():
        Lcd.fillRect(10, 100, 210, 20, BLACK)
        text(10, 100, "BtnA pressed", GREEN)
    if M5.BtnB.wasPressed():
        Lcd.fillRect(10, 125, 210, 20, BLACK)
        text(10, 125, "BtnB pressed", GREEN)
    if M5.BtnC.wasPressed():
        Lcd.fillRect(10, 150, 210, 20, BLACK)
        text(10, 150, "BtnC pressed", GREEN)


# ---------------- main ----------------
def main():
    global current_test, start_time

    M5.begin()
    try:
        Lcd.setRotation(1)
    except Exception:
        pass
    try:
        Lcd.setBrightness(180)
    except Exception:
        pass
    Lcd.setTextColor(WHITE, BLACK)
    Lcd.setTextSize(2)

    # PSRAM check
    draw_header("PSRAM Check:")
    update_results(check_psram())
    time.sleep_ms(2000)
    next_test()  # -> TEST_PORTS (draws no screen yet, like the original)

    # Port setup prompt
    draw_header("Connect the following:")
    text(10, 60, "Angle Unit: Port A")
    text(10, 90, "Dual Button Unit: Port C")
    text(10, 120, "Tap BtnA when ready.")
    while True:
        M5.update()
        if M5.BtnA.wasPressed():
            break
        time.sleep_ms(10)

    start_time = time.ticks_ms()
    setup_ports()
    setup_test_screen()

    while True:
        M5.update()
        poll_touch()

        if current_test == TEST_PORTS:
            loop_ports()
        elif current_test == TEST_LEDS:
            loop_leds()
        elif current_test == TEST_DISPLAY:
            loop_display()
        elif current_test == TEST_TOUCH:
            loop_touch()
        elif current_test == TEST_BUTTONS:
            loop_buttons()
        elif current_test == TEST_IMU:
            imu_cube_frame()
        elif current_test == TEST_MICROPHONE:
            microphone_frame()

        if current_test < TEST_COUNT and next_pressed():
            next_test()
            time.sleep_ms(100)

        time.sleep_ms(10)


try:
    main()
except Exception as e:
    # Show any crash on screen so it can be diagnosed without a serial link.
    import sys
    sys.print_exception(e)
    try:
        Lcd.fillScreen(BLACK)
        text(10, 10, "Test crashed:", RED)
        text(10, 40, e.__class__.__name__, RED, 1)
        text(10, 60, str(e), WHITE, 1)
    except Exception:
        pass
