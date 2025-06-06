#!/usr/bin/env python3
import time
import datetime
import os
import csv
import serial
import re
import threading
import RPi.GPIO as GPIO
from RPLCD.gpio import CharLCD
import board
import busio
import adafruit_bno055

# ──────────────────────────────────────────────────────────────────────────────
# 1) CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────

THRESHOLD_DEG = 4.0       # tilt tolerance for LED/buzzer
HISTORY_WINDOW = 10       # how many past entries to keep
SAVE_MESSAGE_DURATION = 1 # seconds to show “Data saved”
DELETE_MESSAGE_DURATION = 1 # seconds to show “Entry deleted”

GREEN_LED_PIN = 4   # Green LED (tilt OK)
RED_LED_PIN   = 18  # Red LED (tilt)
BUZZER_PIN    = 25  # Buzzer via PWM

# LCD #1 (LiDAR + tilt)
LCD1_PINS = {
    'pin_rs': 26,
    'pin_rw': None,
    'pin_e': 19,
    'pins_data': [13, 6, 5, 11]
}

# LCD #2 (GPS/time/ALT + prompts/history)
LCD2_PINS = {
    'pin_rs': 21,
    'pin_rw': None,
    'pin_e': 20,
    'pins_data': [16, 12, 7, 8]
}

# LiDAR over USB
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 115200

# Arduino Nano over USB (buttons & joystick)
ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_BAUD = 115200

# GPS via Pi hardware UART
GPS_PORT = '/dev/serial0'
GPS_BAUD = 9600

# CSV path (with seconds)
CSV_PATH = os.path.expanduser('~/lidar_data.csv')

# ──────────────────────────────────────────────────────────────────────────────
# 2) NMEA PARSER (GPS)
# ──────────────────────────────────────────────────────────────────────────────
def parse_nmea(sentence):
    fields = sentence.strip().split(',')
    if not fields or not fields[0].startswith('$GP'):
        return {}
    typ = fields[0][3:]
    out = {'type': typ, 'lat': None, 'lon': None, 'alt': None, 'fix': False, 'time_utc': None}
    def conv(raw, hemi):
        if not raw or not hemi:
            return None
        deg = float(raw)//100
        minu = float(raw) - deg*100
        dec = deg + minu/60
        if hemi in ('S','W'):
            dec = -dec
        return dec
    if typ == 'GGA':
        raw_lat, hemi_ns = fields[2], fields[3]
        raw_lon, hemi_ew = fields[4], fields[5]
        fixq = fields[6]
        alt   = fields[9]
        lat = conv(raw_lat, hemi_ns)
        lon = conv(raw_lon, hemi_ew)
        out['lat'] = lat
        out['lon'] = lon
        out['fix'] = (fixq!='' and int(fixq)>0)
        try:
            out['alt'] = float(alt)
        except:
            out['alt'] = None
    elif typ == 'RMC':
        raw_time = fields[1]
        status   = fields[2]
        raw_lat, hemi_ns = fields[3], fields[4]
        raw_lon, hemi_ew = fields[5], fields[6]
        lat = conv(raw_lat, hemi_ns)
        lon = conv(raw_lon, hemi_ew)
        out['lat'] = lat
        out['lon'] = lon
        out['fix'] = (status=='A')
        if raw_time and len(raw_time)>=6:
            hh, mm, ss = raw_time[0:2], raw_time[2:4], raw_time[4:6]
            out['time_utc'] = f"{hh}:{mm}:{ss}"
    return out

# ──────────────────────────────────────────────────────────────────────────────
# 3) TILT DIRECTION HELPER
# ──────────────────────────────────────────────────────────────────────────────
def get_tilt_direction(roll, pitch, thresh=THRESHOLD_DEG):
    if roll is None or pitch is None:
        return "??"
    dirs = []
    if pitch >  thresh: dirs.append("S")
    elif pitch < -thresh: dirs.append("N")
    if roll  >  thresh: dirs.append("W")
    elif roll  < -thresh: dirs.append("E")
    return "OK" if not dirs else "".join(dirs)

# ──────────────────────────────────────────────────────────────────────────────
# 4) GPIO + Device SETUP
# ──────────────────────────────────────────────────────────────────────────────

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# LEDs / buzzer
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN,   GPIO.OUT)
GPIO.setup(BUZZER_PIN,    GPIO.OUT)
GPIO.output(GREEN_LED_PIN, GPIO.LOW)
GPIO.output(RED_LED_PIN,   GPIO.LOW)

# PWM for buzzer at 1 kHz
buzzer_pwm = GPIO.PWM(BUZZER_PIN, 1000)

# LCD #1
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

# LCD #2
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

# IMU (BNO055)
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# LiDAR
try:
    lidar_ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.1)
    time.sleep(0.5)
    lidar_ser.reset_input_buffer()
except:
    lidar_ser = None

# GPS
try:
    gps_ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0.1)
    time.sleep(0.5)
    gps_ser.reset_input_buffer()
except:
    gps_ser = None

# Arduino Nano
try:
    arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
    time.sleep(0.5)
    arduino_ser.reset_input_buffer()
except:
    arduino_ser = None

# Ensure CSV exists
if not os.path.isfile(CSV_PATH):
    with open(CSV_PATH, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Date','Time','Lat','Lon','Alt','Distance_m','Roll','Pitch','GPS_fix'])

saved_entries = []

# Custom “°” char for lcd1
deg_bitmap = [
    0b00111,
    0b01001,
    0b00111,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
]
lcd1.create_char(0, bytearray(deg_bitmap))

# State variables
in_history_mode     = False
history_index       = 0

display_freeze      = False
delete_confirm      = False
delete_prompt_shown = False

# GPS state
gps_lat         = None
gps_lon         = None
gps_alt         = None
gps_fix         = False
gps_time_utc    = None
gps_last_update = 0.0

# Shared for Arduino commands
arduino_command   = None
arduino_timestamp = 0.0
arduino_lock      = threading.Lock()

# ──────────────────────────────────────────────────────────────────────────────
# 5) THREAD: READ ARDUINO (buttons & joystick)
# ──────────────────────────────────────────────────────────────────────────────
def read_arduino():
    global arduino_command, arduino_timestamp
    if not arduino_ser:
        return
    while True:
        line = arduino_ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            time.sleep(0.02)
            continue
        if line in ("SAVE_PRESS", "DELETE_PRESS", "HISTORY", "UP", "DOWN"):
            with arduino_lock:
                if line != arduino_command:
                    arduino_command = line
                    arduino_timestamp = time.time()
        time.sleep(0.01)

if arduino_ser:
    threading.Thread(target=read_arduino, daemon=True).start()

# ──────────────────────────────────────────────────────────────────────────────
# 6) THREAD: READ GPS (NMEA)
# ──────────────────────────────────────────────────────────────────────────────
def read_gps():
    global gps_lat, gps_lon, gps_alt, gps_fix, gps_time_utc, gps_last_update
    if not gps_ser:
        return
    while True:
        line = gps_ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            time.sleep(0.02)
            continue
        data = parse_nmea(line)
        if not data:
            continue
        typ = data['type']
        if typ == 'GGA':
            gps_lat = data['lat']
            gps_lon = data['lon']
            gps_alt = data['alt']
            gps_fix = data['fix']
            gps_last_update = time.time()
        elif typ == 'RMC':
            if data['time_utc']:
                gps_time_utc = data['time_utc']
                gps_last_update = time.time()
        time.sleep(0.01)

if gps_ser:
    threading.Thread(target=read_gps, daemon=True).start()

# ──────────────────────────────────────────────────────────────────────────────
# 7) MAIN LOOP
# ──────────────────────────────────────────────────────────────────────────────

print("Starting integrated_v4.py. Ctrl-C to quit.")
try:
    while True:
        now = time.time()

        # A) READ LiDAR
        distance_str = "--.--m"
        if lidar_ser:
            latest_line = None
            try:
                while lidar_ser.in_waiting:
                    raw = lidar_ser.readline()
                    text = raw.decode('ascii', errors='ignore').strip()
                    if text:
                        latest_line = text
                if latest_line is None:
                    raw = lidar_ser.readline()
                    latest_line = raw.decode('ascii', errors='ignore').strip()
                match = re.search(r'([+-]?\d+(?:\.\d+)?)\s*m', latest_line.replace(',', '.'))
                if match:
                    distance_val = float(match.group(1))
                    distance_str = f"{distance_val:.2f}m"
            except:
                pass

        # B) READ IMU tilt
        euler = sensor.euler
        if euler is None:
            roll, pitch = None, None
        else:
            _, roll, pitch = euler

        if roll is None or pitch is None:
            tilt_cells = [
                " ", " ", chr(0), "x", " ",
                " ", " ", chr(0), "y",
                " ", "T", "I", "L", ":", " ", " "
            ]
        else:
            rx = min(int(abs(roll)), 99)
            ry = min(int(abs(pitch)), 99)
            sx = f"{rx:02d}"
            sy = f"{ry:02d}"
            dir_str = get_tilt_direction(roll, pitch)
            dd = "OK" if dir_str == "OK" else dir_str[:2].ljust(2)
            tilt_cells = [
                sx[0], sx[1], chr(0), "x", " ",
                sy[0], sy[1], chr(0), "y",
                " ", "T", "I", "L", ":", dd[0], dd[1]
            ]

        # C) HANDLE ARDUINO COMMANDS
        with arduino_lock:
            cmd = arduino_command

        if cmd == "SAVE_PRESS":
            with arduino_lock:
                arduino_command = None

            # Cancel delete prompt if pending
            if delete_confirm:
                delete_confirm = False
                delete_prompt_shown = False
                lcd1.clear()

            # FIRST Save: show “Data saved” then freeze
            elif not display_freeze and not in_history_mode:
                lcd1.clear()
                lcd1.write_string("  Data saved    ")

                # Write CSV (include roll/pitch)
                dt = datetime.datetime.now()
                date_str = dt.strftime("%Y-%m-%d")
                # TIME with seconds from GPS if possible, else system
                if gps_fix and gps_time_utc:
                    hh, mm, ss = map(int, gps_time_utc.split(':'))
                    now_utc = datetime.datetime.utcnow().replace(hour=hh, minute=mm, second=ss)
                    now_local = now_utc.astimezone()
                    time_str = now_local.strftime("%I:%M:%S %p")
                else:
                    time_str = datetime.datetime.now().strftime("%I:%M:%S %p")

                lat  = gps_lat if gps_fix else None
                lon  = gps_lon if gps_fix else None
                alt  = gps_alt if gps_fix else None
                fix  = "Yes" if gps_fix else "No"
                roll_val  = f"{roll:.1f}"  if roll is not None else ""
                pitch_val = f"{pitch:.1f}" if pitch is not None else ""
                dist_val  = distance_str.rstrip('m')

                with open(CSV_PATH, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        date_str, time_str,
                        f"{lat:.6f}" if lat else "",
                        f"{lon:.6f}" if lon else "",
                        f"{alt:.1f}" if alt else "",
                        dist_val, roll_val, pitch_val, fix
                    ])
                saved_entries.insert(0, [
                    date_str, time_str,
                    f"{lat:.6f}" if lat else "",
                    f"{lon:.6f}" if lon else "",
                    f"{alt:.1f}" if alt else "",
                    dist_val, roll_val, pitch_val, fix
                ])
                if len(saved_entries) > HISTORY_WINDOW:
                    saved_entries.pop()

                time.sleep(SAVE_MESSAGE_DURATION)
                display_freeze = True

            # SECOND Save (while frozen): unfreeze
            elif display_freeze:
                display_freeze = False

        elif cmd == "DELETE_PRESS":
            with arduino_lock:
                arduino_command = None

            # In history: prompt+confirm
            if in_history_mode:
                if not delete_confirm:
                    delete_confirm = True
                    lcd2.clear()
                    lcd2.cursor_pos = (0,0)
                    lcd2.write_string("Press delete".center(16))
                    lcd2.cursor_pos = (1,0)
                    lcd2.write_string("again to delete".center(16))
                else:
                    # Perform deletion
                    if saved_entries:
                        saved_entries.pop(history_index)
                        with open(CSV_PATH, 'w', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(['Date','Time','Lat','Lon','Alt','Distance_m','Roll','Pitch','GPS_fix'])
                            writer.writerows(saved_entries)
                    delete_confirm = False
                    lcd2.clear()
                    lcd2.cursor_pos = (0,0)
                    lcd2.write_string(" Entry deleted  ")
                    time.sleep(DELETE_MESSAGE_DURATION)
                    lcd2.clear()
                    # Clamp history_index
                    history_index = min(history_index, len(saved_entries)-1)
                    if not saved_entries:
                        in_history_mode = False
                    lcd1.clear()

            # Not frozen/history: prompt deletion of most recent
            elif not display_freeze and not in_history_mode:
                if not delete_confirm:
                    delete_confirm = True
                    delete_prompt_shown = True
                    lcd2.clear()
                    lcd2.cursor_pos = (0,0)
                    lcd2.write_string("Press delete".center(16))
                    lcd2.cursor_pos = (1,0)
                    lcd2.write_string("again to delete".center(16))
                else:
                    if saved_entries:
                        saved_entries.pop(0)
                        with open(CSV_PATH, 'w', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(['Date','Time','Lat','Lon','Alt','Distance_m','Roll','Pitch','GPS_fix'])
                            writer.writerows(saved_entries)
                    delete_confirm = False
                    delete_prompt_shown = False
                    lcd2.clear()
                    lcd2.cursor_pos = (0,0)
                    lcd2.write_string(" Entry deleted  ")
                    time.sleep(DELETE_MESSAGE_DURATION)
                    lcd2.clear()

        elif cmd == "HISTORY":
            with arduino_lock:
                arduino_command = None
            if not in_history_mode and not display_freeze:
                in_history_mode = True
                history_index = 0
                lcd1.clear()
                lcd1.cursor_pos = (0,0)
                lcd1.write_string(" History Mode    ")
                time.sleep(0.5)
                lcd1.clear()
            elif in_history_mode:
                in_history_mode = False
                lcd1.clear()

        elif cmd == "UP":
            with arduino_lock:
                arduino_command = None
            if in_history_mode and (history_index + 1) < len(saved_entries):
                history_index += 1
                lcd1.clear()

        elif cmd == "DOWN":
            with arduino_lock:
                arduino_command = None
            if in_history_mode and history_index > 0:
                history_index -= 1
                lcd1.clear()

        # D) HISTORY MODE DISPLAY
        if in_history_mode:
            if saved_entries:
                ent = saved_entries[history_index]
                # ent = [date, time, lat, lon, alt, dist, roll, pitch, fix]
                # Top row: show time with seconds + distance
                time_str = ent[1]        # “HH:MM:SS PM”
                dist_ent = ent[5]        # e.g. “123.45”
                lcd1.cursor_pos = (0,0)
                lcd1.write_string(time_str.ljust(16)[:16])
                lcd1.cursor_pos = (1,0)
                lcd1.write_string(f"{dist_ent}m".ljust(16)[:16])
                # LCD #2 shows the static text “History (press history to exit)”
                lcd2.clear()
                lcd2.cursor_pos = (0,0)
                lcd2.write_string("History (press".ljust(16)[:16])
                lcd2.cursor_pos = (1,0)
                lcd2.write_string("history to exit)".ljust(16)[:16])
            else:
                lcd1.cursor_pos = (0,0)
                lcd1.write_string("No history       ")
                lcd1.cursor_pos = (1,0)
                lcd1.write_string(" entries         ")
                lcd2.clear()
            time.sleep(0.05)
            continue

        # E) FREEZE OR DELETE-PROMPT LOGIC
        if display_freeze or delete_prompt_shown:
            # LCD #1 remains at last live measurement (no overwrite here)

            # LCD #2 for Save-freeze: show only “Continue?”; for delete prompt, do nothing (already shown)
            if display_freeze:
                lcd2.clear()
                lcd2.cursor_pos = (0,0)
                lcd2.write_string("Continue?".center(16))
                lcd2.cursor_pos = (1,0)
                lcd2.write_string("".center(16))
            # If delete_prompt_shown, we already displayed the prompt above. Just wait.
            time.sleep(0.05)
            continue

        # F) NORMAL LIVE DISPLAY

        # F1) LCD #1 – Top: “Dist:xxx.xxm”
        lcd1.cursor_pos = (0,0)
        lcd1.write_string(f"Dist:{distance_str}".ljust(16)[:16])

        # F2) LCD #1 – Bottom: tilt_cells
        lcd1.cursor_pos = (1,0)
        lcd1.write_string("".join(tilt_cells))

        # F3) LCD #2 – GPS/time/ALT
        lcd2.cursor_pos = (0,0)
        if (time.time() - gps_last_update) > 2.0 or not gps_fix:
            lcd2.write_string("No GPS Signal   ")
            lcd2.cursor_pos = (1,0)
            lcd2.write_string("AL:------       ")
        else:
            # Use GPS time if possible
            if gps_fix and gps_time_utc:
                hh, mm, ss = map(int, gps_time_utc.split(':'))
                now_utc = datetime.datetime.utcnow().replace(hour=hh, minute=mm, second=ss)
                now_local = now_utc.astimezone()
                time_str = now_local.strftime("%I:%M:%S %p")
            else:
                time_str = datetime.datetime.now().strftime("%I:%M:%S %p")

            lat_fmt = f"{gps_lat:.3f}"
            field0 = lat_fmt.ljust(7)    # cols 0–6
            field1 = time_str.rjust(8)   # cols 8–15
            lcd2.write_string(field0 + field1)

            lcd2.cursor_pos = (1,0)
            lon_fmt = f"{gps_lon:.3f}"
            field0 = lon_fmt.ljust(7)
            alt_fmt = f"AL:{gps_alt:.1f}"
            field1 = alt_fmt.rjust(8)
            lcd2.write_string(field0 + field1)

        # F4) LEDs & buzzer
        if roll is not None and pitch is not None and abs(roll) <= THRESHOLD_DEG and abs(pitch) <= THRESHOLD_DEG:
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            GPIO.output(RED_LED_PIN,   GPIO.LOW)
            buzzer_pwm.stop()
        else:
            GPIO.output(GREEN_LED_PIN, GPIO.LOW)
            GPIO.output(RED_LED_PIN,   GPIO.HIGH)
            try:
                buzzer_pwm.start(50)
            except:
                pass

        time.sleep(0.01)

except KeyboardInterrupt:
    pass

finally:
    lcd1.clear()
    lcd2.clear()
    GPIO.output(GREEN_LED_PIN, GPIO.LOW)
    GPIO.output(RED_LED_PIN,   GPIO.LOW)
    buzzer_pwm.stop()
    GPIO.cleanup()
    if lidar_ser:
        lidar_ser.close()
    if gps_ser:
        gps_ser.close()
    if arduino_ser:
        arduino_ser.close()
    print("\nExiting cleanly.")
