#!/usr/bin/env python3
import time
import datetime
import os
import csv
import serial
import RPi.GPIO as GPIO
from RPLCD.gpio import CharLCD
import board
import busio
import adafruit_bno055

# ──────────────────────────────────────────────────────────────────────────────
# 1) CONFIGURATION & PIN ASSIGNMENTS (BCM MODE)
# ──────────────────────────────────────────────────────────────────────────────

THRESHOLD_DEG = 4.0    # tilt tolerance in degrees

# GPIO pins (BCM) for buttons
SAVE_BUTTON_PIN   = 17  # Save / Continue
DELETE_BUTTON_PIN = 27  # Delete last entry

# GPIO pins (BCM) for LEDs and buzzer
GREEN_LED_PIN = 4   # Green LED
RED_LED_PIN   = 18  # Red LED
BUZZER_PIN    = 25  # Active buzzer

# LCD #1 (top display) wiring in BCM
LCD1_PINS = {
    'pin_rs': 26,                # BCM 26
    'pin_rw': None,              # RW tied to GND
    'pin_e': 19,                 # BCM 19
    'pins_data': [13, 6, 5, 11]  # D4→BCM 13, D5→BCM 6, D6→BCM 5, D7→BCM 11
}

# LCD #2 (bottom display) wiring in BCM
LCD2_PINS = {
    'pin_rs': 21,                # BCM 21
    'pin_rw': None,              # RW tied to GND
    'pin_e': 20,                 # BCM 20
    'pins_data': [16, 12, 7, 8]  # D4→BCM 16, D5→BCM 12, D6→BCM 7, D7→BCM 8
}

# LiDAR serial port (adjust if yours is /dev/ttyACM0 instead)
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 115200

# CSV file path
CSV_PATH = os.path.expanduser('~/lidar_data.csv')

# Fixed GPS values for this test
TEST_LAT = '100.420'
TEST_LON = '420.100'
TEST_ALT = '120.4'
GPS_FIX  = 'Yes'

# ──────────────────────────────────────────────────────────────────────────────
# 2) HELPER: TILT DIRECTION
# ──────────────────────────────────────────────────────────────────────────────
def get_tilt_direction(roll, pitch, thresh=THRESHOLD_DEG):
    """
    Return:
      - "OK" if |roll| and |pitch| ≤ thresh
      - Otherwise, combination of N/S (pitch) + E/W (roll) (e.g. "N", "SW", etc.)
      - "??" if sensor not ready
    """
    if roll is None or pitch is None:
        return "??"
    dirs = []
    if pitch >  thresh:
        dirs.append("S")
    elif pitch < -thresh:
        dirs.append("N")
    if roll  >  thresh:
        dirs.append("W")
    elif roll  < -thresh:
        dirs.append("E")
    return "OK" if not dirs else "".join(dirs)

# ──────────────────────────────────────────────────────────────────────────────
# 3) SETUP: GPIO, LCDs, IMU, LiDAR, CSV
# ──────────────────────────────────────────────────────────────────────────────

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Buttons
GPIO.setup(SAVE_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DELETE_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# LEDs and buzzer
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN,   GPIO.OUT)
GPIO.setup(BUZZER_PIN,    GPIO.OUT)

GPIO.output(GREEN_LED_PIN, GPIO.LOW)
GPIO.output(RED_LED_PIN,   GPIO.LOW)
GPIO.output(BUZZER_PIN,    GPIO.LOW)

# LCD #1 (top)
lcd1 = CharLCD(
    pin_rs=LCD1_PINS['pin_rs'],
    pin_rw=LCD1_PINS['pin_rw'],
    pin_e=LCD1_PINS['pin_e'],
    pins_data=LCD1_PINS['pins_data'],
    numbering_mode=GPIO.BCM,
    cols=16, rows=2,
    auto_linebreaks=False
)
lcd1.clear()

# LCD #2 (bottom)
lcd2 = CharLCD(
    pin_rs=LCD2_PINS['pin_rs'],
    pin_rw=LCD2_PINS['pin_rw'],
    pin_e=LCD2_PINS['pin_e'],
    pins_data=LCD2_PINS['pins_data'],
    numbering_mode=GPIO.BCM,
    cols=16, rows=2,
    auto_linebreaks=False
)
lcd2.clear()

# IMU (BNO055) over I²C
i2c = busio.I2C(board.SCL, board.SDA)    # SCL=BCM 3, SDA=BCM 2
sensor = adafruit_bno055.BNO055_I2C(i2c)

# LiDAR serial
try:
    lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
    time.sleep(0.5)
except Exception:
    lidar_ser = None

# CSV setup: write header if new
if not os.path.isfile(CSV_PATH):
    with open(CSV_PATH, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Date','Time','Lat','Lon','Alt','Distance_m','GPS_fix'])

saved_entries = []  # for in-memory deletions

# Custom “°” char for lcd1 line 2 (CGRAM 0)
deg_bitmap = [
    0b00111,  # ..○○○
    0b01001,  # .○..○
    0b00111,  # ..○○○
    0b00000,  # .....
    0b00000,  # .....
    0b00000,  # .....
    0b00000,  # .....
    0b00000   # .....
]
lcd1.create_char(0, bytearray(deg_bitmap))

# Button state memory
prev_save_state   = GPIO.input(SAVE_BUTTON_PIN)
prev_delete_state = GPIO.input(DELETE_BUTTON_PIN)

display_freeze     = False
delete_confirm     = False
delete_prompt_shown = False

# ──────────────────────────────────────────────────────────────────────────────
# 4) MAIN LOOP
# ──────────────────────────────────────────────────────────────────────────────

print("Starting integrated test (fixed). Ctrl-C to quit.")
try:
    while True:
        # --- 1) Read LiDAR distance (or placeholder) ---
        distance_str = "--.--m"
        if lidar_ser:
            raw = lidar_ser.readline()
            try:
                line = raw.decode('ascii', errors='ignore').strip()
                # Expect something like "12.34m 4.567V 100%"
                parts = line.replace(" m","m").split()
                if parts and parts[0].endswith('m'):
                    distance_str = parts[0]
            except:
                pass

        # --- 2) Read IMU tilt (roll & pitch) ---
        euler = sensor.euler   # (heading, roll, pitch) or (None,None,None)
        if euler is None:
            roll = None
            pitch = None
        else:
            _, roll, pitch = euler

        if roll is None or pitch is None:
            # Show placeholders on line 2 of lcd1
            tilt_cells = [
                " ", " ", chr(0), "x",    # “  ° x”
                " ",                         # space between x and y
                " ", " ", chr(0), "y",    # “  ° y”
                " ", "T", "I", "L", ":", " ", " ", " "
            ]
        else:
            rx = min(int(abs(roll)), 99)
            ry = min(int(abs(pitch)), 99)
            sx = f"{rx:02d}"   # e.g. "14"
            sy = f"{ry:02d}"   # e.g. "16"
            dir_str = get_tilt_direction(roll, pitch)
            dd = "OK" if dir_str == "OK" else dir_str[:2].ljust(2)

            # Now place a space between “x” and the “y” digits:
            tilt_cells = [
                sx[0], sx[1], chr(0), "x",  # e.g. “1 4 ° x”
                " ",                         # explicit space
                sy[0], sy[1], chr(0), "y",  # “ 1 6 ° y”
                " ", "T", "I", "L", ":",    # “   T I L :”
                dd[0], dd[1], " "            # direction + trailing space
            ]

        # --- 3) Handle Buttons ---
        curr_save_state   = GPIO.input(SAVE_BUTTON_PIN)
        curr_delete_state = GPIO.input(DELETE_BUTTON_PIN)

        # SAVE / CONTINUE button pressed (HIGH→LOW transition)
        if curr_save_state == GPIO.LOW and prev_save_state == GPIO.HIGH:
            if not delete_confirm:
                if not display_freeze:
                    # === Save current entry to CSV ===
                    now = datetime.datetime.now()
                    date_str = now.strftime("%Y-%m-%d")
                    time_str = now.strftime("%I:%M %p")  # e.g. “12:49 PM”
                    lat = TEST_LAT
                    lon = TEST_LON
                    alt = TEST_ALT
                    dist_val = distance_str.rstrip('m')
                    fix = GPS_FIX

                    with open(CSV_PATH, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([date_str, time_str, lat, lon, alt, dist_val, fix])
                    saved_entries.append([date_str, time_str, lat, lon, alt, dist_val, fix])

                    # Show “Data saved” on lcd1, then freeze
                    lcd1.clear()
                    lcd1.write_string("  Data saved    ")
                    display_freeze = True
                else:
                    # Second press → unfreeze
                    display_freeze = False

        # DELETE button pressed
        if curr_delete_state == GPIO.LOW and prev_delete_state == GPIO.HIGH:
            if not display_freeze:
                if not delete_confirm:
                    # Show confirmation prompt
                    lcd1.clear()
                    lcd1.write_string("Delete last entry?")
                    delete_confirm = True
                    delete_prompt_shown = True
                else:
                    # Confirm deletion
                    if saved_entries:
                        saved_entries.pop()
                        # Rewrite CSV without last row
                        with open(CSV_PATH, 'w', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(['Date','Time','Lat','Lon','Alt','Distance_m','GPS_fix'])
                            writer.writerows(saved_entries)
                    lcd1.clear()
                    lcd1.write_string("  Entry deleted ")
                    delete_confirm = False
                    delete_prompt_shown = False

        prev_save_state   = curr_save_state
        prev_delete_state = curr_delete_state

        # If freeze or delete‐prompt, skip normal updates but still update lcd2:
        if display_freeze or delete_prompt_shown:
            # Always update LCD #2 with test GPS/time/alt so it doesn't go blank:
            now = datetime.datetime.now()
            time_str = now.strftime("%I:%M %p")
            lcd2.cursor_pos = (0, 0)
            row0 = f"{TEST_LAT} {time_str}"
            lcd2.write_string(row0.ljust(16)[:16])
            lcd2.cursor_pos = (1, 0)
            row1 = f"{TEST_LON} AL:{TEST_ALT}"
            lcd2.write_string(row1.ljust(16)[:16])

            time.sleep(0.05)
            continue

        # --- 4) Normal Display Update ---

        # LCD #1 – Top row: “Dist:xxx.xxm”
        lcd1.cursor_pos = (0, 0)
        dist_display = f"Dist:{distance_str}"
        lcd1.write_string(dist_display.ljust(16)[:16])

        # LCD #1 – Bottom row: tilt_cells (16 chars)
        lcd1.cursor_pos = (1, 0)
        lcd1.write_string("".join(tilt_cells))

        # LCD #2 – always update:
        now = datetime.datetime.now()
        time_str = now.strftime("%I:%M %p")
        lcd2.cursor_pos = (0, 0)
        row0 = f"{TEST_LAT} {time_str}"
        lcd2.write_string(row0.ljust(16)[:16])

        lcd2.cursor_pos = (1, 0)
        row1 = f"{TEST_LON} AL:{TEST_ALT}"
        lcd2.write_string(row1.ljust(16)[:16])

        # LEDs & Buzzer:
        if roll is not None and pitch is not None and abs(roll) <= THRESHOLD_DEG and abs(pitch) <= THRESHOLD_DEG:
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            GPIO.output(RED_LED_PIN,   GPIO.LOW)
            GPIO.output(BUZZER_PIN,    GPIO.LOW)
        else:
            GPIO.output(GREEN_LED_PIN, GPIO.LOW)
            GPIO.output(RED_LED_PIN,   GPIO.HIGH)
            GPIO.output(BUZZER_PIN,    GPIO.HIGH)  # Active buzzer on

        # Tiny delay to let CPU breathe; LiDAR now updates quickly
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

finally:
    lcd1.clear()
    lcd2.clear()
    GPIO.output(GREEN_LED_PIN, GPIO.LOW)
    GPIO.output(RED_LED_PIN,   GPIO.LOW)
    GPIO.output(BUZZER_PIN,    GPIO.LOW)
    GPIO.cleanup()
    if lidar_ser:
        lidar_ser.close()
    print("\nExiting cleanly.")
