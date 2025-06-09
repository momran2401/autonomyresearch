Mobile LiDAR Data Logger

© 2025 Mustafa Omran. All rights reserved.

This project is a fully integrated field data‑logging and feedback system running on a Raspberry Pi. It interfaces with a LiDAR distance sensor, IMU (BNO055), GPS receiver, and an Arduino Nano to offer real‑time measurement display, data storage, and user controls via buttons, joystick, LEDs, buzzer, and dual LCD screens.

Features

Real‑time display of distance, tilt (roll & pitch), GPS coordinates, altitude, and time on two 16×2 LCDs

Visual & audible feedback: green/red LEDs and buzzer indicate level vs. tilt

Data logging: save readings (date, time, latitude, longitude, altitude, distance, roll, pitch, GPS fix) to CSV

History browser: scroll through, review, and delete past entries

Shutdown: hold joystick fully left/right for 3 s to power off (cancelable)

Startup hardware checks with animated progress and error reporting

Hardware Requirements & Wiring Summary

All pin references use BCM numbering on the Raspberry Pi 3 B. Arduino pins use Nano labeling.

1. LCD #1 (LiDAR + Tilt)

VSS → GND
VDD → 5 V
V0  → wiper of 10 KΩ pot (ends to 5 V & GND)
RS  → BCM 26
RW  → GND
E   → BCM 19
D4  → BCM 13
D5  → BCM 6
D6  → BCM 5
D7  → BCM 11
A   → 5 V via 220 Ω resistor
K   → GND

2. LCD #2 (GPS/Time/Alt + Prompts)

VSS → GND
VDD → 5 V
V0  → wiper of 10 KΩ pot
RS  → BCM 21
RW  → GND
E   → BCM 20
D4  → BCM 16
D5  → BCM 12
D6  → BCM 7
D7  → BCM 8
A   → 5 V via 220 Ω resistor
K   → GND

3. LEDs & Buzzer

Green LED (+) → BCM 4 via 330 Ω → LED (–) → GND
Red   LED (+) → BCM 18 via 330 Ω → LED (–) → GND
Buzzer (+)    → BCM 25 → Buzzer (–) → GND

4. IMU (BNO055 over I2C)

VIN → 3.3 V
GND → GND
SDA → BCM 2 (pin 3)
SCL → BCM 3 (pin 5)

5. GPS (Adafruit Ultimate v3 via UART)

VIN → 5 V
GND → GND
TX  → BCM 15 (UART RX)
RX  → (leave unconnected)

6. Arduino Nano (Buttons & Joystick)

D2  → Save/Continue button (active LOW)
D3  → Delete last entry button (active LOW)
D4  → Joystick button (“History”, active LOW)
A0  → Joystick X (shutdown when held left/right)
A1  → Joystick Y (UP/DOWN navigation)
5 V → Joystick VCC
GND → Joystick GND
USB → Pi USB port (for serial comm at 115200 baud)

7. LiDAR (SF11/C)

USB micro‑B → Pi USB‑A port (/dev/ttyUSB0)

Software Installation

Install system packages:

sudo apt update
sudo apt install python3-venv python3-pip i2c-tools

Create & activate Python venv:

python3 -m venv ~/imu-env
source ~/imu-env/bin/activate

Install Python deps:

pip install RPLCD pyserial adafruit-blinka adafruit-circuitpython-bno055 RPi.GPIO

Enable I2C & serial:

sudo raspi-config
# Interfacing Options → I2C → Enable
# Interfacing Options → Serial → disable login shell, enable serial port

Running the Application

Manual Launch

source ~/imu-env/bin/activate
python ~/integratedv12.py

Automatic on Boot (systemd)

Service file /etc/systemd/system/lidar.service:

[Unit]
Description=Mobile LiDAR Unit
After=network.target

[Service]
User=mustafaomran
WorkingDirectory=/home/mustafaomran
ExecStart=/home/mustafaomran/imu-env/bin/python3 /home/mustafaomran/integratedv12.py
Restart=always
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target

Enable & start:

sudo systemctl daemon-reload
sudo systemctl enable lidar.service
sudo systemctl start lidar.service
sudo journalctl -u lidar.service -f

User Controls (Arduino Commands)

Command

Trigger

Effect

SAVE_PRESS

Press Save button (D2)

Save entry, show “Data saved” → freeze display

DELETE_PRESS

Press Delete button (D3)

Prompt & confirm deletion (in live/history modes)

HISTORY

Press Joystick button (D4)

Toggle history browsing mode

UP

Joystick up (A1 < 400)

Scroll older entries in history

DOWN

Joystick down (A1 > 600)

Scroll newer entries in history

SHUTDOWN_PRESS

Hold joystick X left/right (A0<400/>600) 3 s

Countdown & power off (cancelable)

Data Logging

CSV file: ~/lidar_data.csv with columns:

Date,Time,Lat,Lon,Alt,Distance_m,Roll,Pitch,GPS_fix

History buffer: keeps last HISTORY_WINDOW entries in memory

README & License Notice

This repository and its contents (code, schematics, documentation) are © 2025 Mustafa Omran. All rights reserved.

No portion of this project may be used, copied, distributed, or incorporated into any product — commercial or otherwise — without the prior written permission of the author.

Contact: mustafaomran@example.com
