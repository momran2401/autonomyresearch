# Mobile LiDAR Data Logger

**© 2025 Mustafa Omran. All rights reserved.**

A Raspberry Pi 3B–based field data logger integrating LiDAR, IMU, GPS, and user controls via an Arduino Nano. Provides real-time feedback, data logging, and history browsing.

---

## Features

* **Real-time display** on two 16×2 LCDs:

  * Distance (LiDAR)
  * Tilt: roll & pitch (IMU)
  * Latitude / longitude, altitude, timestamp (GPS)
* **Visual & audible feedback**:

  * Green/red LEDs
  * PWM buzzer
* **Data logging** to CSV:

  ```csv
  Date,Time,Lat,Lon,Alt,Distance_m,Roll,Pitch,GPS_fix
  ```
* **History mode** (up to 100 entries): scroll, review, delete
* **Shutdown** via joystick hold (3 s) with cancel
* **Startup hardware checks** with animated status and error reporting

---

## Wiring Summary (BCM / Nano)

### LCD #1 (LiDAR + Tilt)

| Pin | BCM           | Function                 |
| --- | ------------- | ------------------------ |
| VSS | GND           | Ground                   |
| VDD | 5 V           | Power                    |
| V0  | Pot →GND/5 V  | Contrast                 |
| RS  | 26            | Register Select          |
| RW  | GND           | Read/Write (GND = write) |
| E   | 19            | Enable                   |
| D4  | 13            | Data 4                   |
| D5  | 6             | Data 5                   |
| D6  | 5             | Data 6                   |
| D7  | 11            | Data 7                   |
| A   | 5 V via 220 Ω | Backlight +              |
| K   | GND           | Backlight –              |

### LCD #2 (GPS/Time/Alt + Prompts)

| Pin | BCM           | Function        |
| --- | ------------- | --------------- |
| VSS | GND           | Ground          |
| VDD | 5 V           | Power           |
| V0  | Pot           | Contrast        |
| RS  | 21            | Register Select |
| RW  | GND           | Read/Write      |
| E   | 20            | Enable          |
| D4  | 16            | Data 4          |
| D5  | 12            | Data 5          |
| D6  | 7             | Data 6          |
| D7  | 8             | Data 7          |
| A   | 5 V via 220 Ω | Backlight +     |
| K   | GND           | Backlight –     |

### LEDs & Buzzer

| Device    | Pin    | Details            |
| --------- | ------ | ------------------ |
| Green LED | BCM 4  | via 330 Ω → GND    |
| Red LED   | BCM 18 | via 330 Ω → GND    |
| Buzzer    | BCM 25 | PWM @1 kHz to beep |

### IMU (BNO055, I²C)

| Pin | BCM   | Function  |
| --- | ----- | --------- |
| VIN | 3.3 V | Power     |
| GND | GND   | Ground    |
| SDA | 2     | I²C Data  |
| SCL | 3     | I²C Clock |

### GPS (Adafruit Ultimate v3, UART)

| Pin | BCM | Function     |
| --- | --- | ------------ |
| VIN | 5 V | Power        |
| GND | GND | Ground       |
| TX  | 15  | Pi RX (UART) |
| RX  | —   | (Unused)     |

### Arduino Nano & Joystick

| Function              | Nano Pin | Notes                               |
| --------------------- | -------- | ----------------------------------- |
| Save / Continue       | D2       | Button (active LOW)                 |
| Delete Entry          | D3       | Button (active LOW)                 |
| History Toggle        | D4       | Joystick SW (active LOW)            |
| Joystick X (shutdown) | A0       | ADC <400/< >600 hold for shutdown   |
| Joystick Y (nav)      | A1       | ADC <400=UP, >600=DOWN              |
| 5 V                   | 5 V      | Joystick VCC                        |
| GND                   | GND      | Joystick ground                     |
| USB Serial            | USB      | To Pi (/dev/ttyACM0 at 115200 baud) |

### LiDAR (SF11/C)

| Interface | Details                           |
| --------- | --------------------------------- |
| USB       | Micro-B → Pi USB-A (/dev/ttyUSB0) |

---

## Software Installation

```bash
sudo apt update
sudo apt install python3-venv python3-pip i2c-tools
python3 -m venv ~/imu-env
source ~/imu-env/bin/activate
pip install RPLCD pyserial adafruit-blinka adafruit-circuitpython-bno055 RPi.GPIO
sudo raspi-config   # enable I2C & UART
```

Clone and prepare repo:

```bash
git clone https://github.com/yourusername/lidar-logger.git
cd lidar-logger
chmod +x integratedv12.py
```

---

## Running the App

**Manual**:

```bash
source ~/imu-env/bin/activate
python integratedv12.py
```

**Auto‑start on Boot (systemd)**:

1. Create `/etc/systemd/system/lidar.service`:

   ```ini
   [Unit]
   Description=Mobile LiDAR Unit
   After=network.target

   [Service]
   User=mustafaomran
   WorkingDirectory=/home/mustafaomran/lidar-logger
   ExecStart=/home/mustafaomran/imu-env/bin/python3 /home/mustafaomran/lidar-logger/integratedv12.py
   Restart=always
   Environment=PYTHONUNBUFFERED=1

   [Install]
   WantedBy=multi-user.target
   ```
2. Enable & start:

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable lidar.service
   sudo systemctl start lidar.service
   sudo journalctl -u lidar.service -f
   ```

---

## User Controls (Serial Commands)

| Command             | Trigger                         | Effect                               |
| ------------------- | ------------------------------- | ------------------------------------ |
| **SAVE\_PRESS**     | D2 button press                 | Save reading → “Data saved” → freeze |
| **DELETE\_PRESS**   | D3 button press                 | Prompt & confirm deletion            |
| **HISTORY**         | Joystick SW (D4) press          | Toggle history mode                  |
| **UP**              | Joystick pushed up (A1 < 400)   | Scroll older history                 |
| **DOWN**            | Joystick pushed down (A1 > 600) | Scroll newer history                 |
| **SHUTDOWN\_PRESS** | Hold joystick X left/right 3 s  | Countdown & shutdown                 |

---

## Data Logging Format

* **File**: `~/lidar_data.csv`
* **Columns**:

  ```csv
  Date,Time,Lat,Lon,Alt,Distance_m,Roll,Pitch,GPS_fix
  ```
* **In‑memory history**: last 100 entries (browsable)

---

## Troubleshooting & Health Checks

* **Startup checks**: animated “Running checks…” + error report
* **Runtime**:

  * GPS no data (>10 s): “GPS no data – Check antenna”
  * No IMU: “IMU disconnected”
  * No LiDAR: “LiDAR missing”
  * Joystick/Nano no input (>5 s): “Joystick Missing”
* **Logs**: `sudo journalctl -u lidar.service -f`

---

## Copyright Notice

**Copyright © 2025 Mustafa Omran**

This project — *in whole or in part* — including but not limited to all **source code**, **scripts**, **logic**, **wiring diagrams**, **system architecture**, **hardware-software integration**, **feature concepts**, and **written documentation** (collectively, **"the Work"**) — is the sole intellectual property of **Mustafa Omran**.

Unauthorized use, reproduction, modification, sublicensing, or distribution of any portion of this Work is **strictly prohibited** without the **express written consent** of the copyright holder. This includes attempts to:
- Repurpose the design or codebase in any derivative or competing product.  
- Reproduce or reuse any part of the wiring, hardware configuration, or feature logic.  
- Incorporate this Work — *in part or in full* — into a separate project, publication, or system (academic, commercial, or governmental) without proper **attribution** and **authorization**.

**Note:** Public availability of this Work — including through platforms like GitHub — does **not** constitute a waiver of rights or permission for derivative use.

All rights are **explicitly reserved** to the creator, regardless of employment status, contractual obligations, or employer assumptions. This declaration is reinforced by timestamped authorship in GitHub commit history, File creation and modification records, Clear authorship attribution in project metadata. The author reserves the right to pursue **legal action** in the event of **misappropriation**, **unauthorized replication**, or **unlicensed commercial use** of the Work. Third-party components (e.g. physical casings or enclosures) are **not** covered by this claim unless explicitly stated.

Contact: [momran2401@gmail.com](mailto:momran2401@gmail.com)

---

*End of README*
