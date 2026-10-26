# ESP32-Based Orthostatic Response and Vital Monitoring System

An embedded physiological monitoring prototype integrating **ESP32, MAX30102, MPU6050, Qt/QML, Wi-Fi communication, FreeRTOS-based notification handling, and KiCad hardware design**.

The project combines heart-rate and SpO₂ estimation, posture classification, resting-baseline acquisition, standing-response monitoring, and event-based alerts in a connected monitoring platform.

> **Project status:** Prototype under development. USB-powered operation has been demonstrated. Battery-powered operation requires further validation.

---

## Overview

The ESP32-Based Orthostatic Response and Vital Monitoring System is designed to demonstrate how embedded electronics, physiological sensors, real-time processing, and desktop visualization can be combined into a connected monitoring prototype.

The ESP32 acquires sensor data from the MAX30102 and MPU6050, processes the readings, manages the monitoring state machine, and transmits structured JSON messages over Wi-Fi TCP to a Qt-based graphical interface.

The system also supports user-triggered dizziness reporting and configurable notifications through the ntfy service.

The hardware design documentation includes the circuit architecture, component connections, and KiCad design files as they become available.

## Key Features

- **Heart-rate monitoring:** Estimates heart rate from MAX30102 optical pulse signals.
- **SpO₂ estimation:** Processes red and infrared sensor data to estimate blood oxygen saturation.
- **Posture classification:** Uses MPU6050 acceleration measurements and calibration-based posture references.
- **Resting baseline acquisition:** Collects valid heart-rate samples during a configured resting period.
- **Standing-response monitoring:** Observes heart-rate changes during a three-minute standing test.
- **Configurable response alerts:** Detects a sustained heart-rate increase relative to the resting baseline.
- **Qt/QML dashboard:** Displays sensor readings, posture, monitoring state, and alerts.
- **Wi-Fi TCP communication:** Transfers JSON-formatted data between the ESP32 and the Qt application.
- **Dizziness-event reporting:** Supports user-triggered events through available input methods.
- **Remote notifications:** Integrates with ntfy for notification delivery when network connectivity is available.
- **Battery-powered operation:** Includes a TP4056 charging module and MT3608 boost converter; the power system requires further validation.
- **KiCad hardware design:** Provides a place for schematic, PCB, and component documentation.

---

## System Architecture

```text
              MAX30102
          Heart Rate / SpO₂
                  |
                  | I²C
                  |
                  v
             +---------+
             |         |
             |  ESP32  | <----- MPU6050
             | DevKit  |        Acceleration /
             |         |        Posture Data
             +---------+
                  |
          Sensor Processing
          Posture Classification
          Monitoring State Machine
                  |
          +-------+--------+
          |                |
          v                v
     Wi-Fi TCP         Notification
     JSON Messages     Processing Task
          |                |
          v                v
      Qt/QML GUI       ntfy Service
          |
          v
   Monitoring Display
   and Event Reporting
```

## Hardware Components

| Component | Purpose |
|---|---|
| ESP32 DevKit V1 | Main microcontroller and Wi-Fi communication |
| MAX30102 | Optical heart-rate and SpO₂ sensing |
| MPU6050 | Accelerometer and gyroscope for posture-related measurements |
| TP4056 module | Single-cell lithium battery charging |
| 3.7 V, 400 mAh battery | Portable power source |
| MT3608 boost converter | Steps battery voltage up to a regulated output |
| LEDs and resistors | Visual status indicators |
| Active buzzer | Audible event indication, when installed and integrated |
| Computer | Qt dashboard, firmware development, and testing |

Component specifications and wiring must be checked against the actual hardware modules used.

## Software and Technologies

| Technology | Application |
|---|---|
| C/C++ | ESP32 firmware |
| Arduino framework | Firmware development |
| ESP32 Arduino Core | Microcontroller support |
| MAX3010x sensor library | MAX30102 interface |
| I²C | Sensor communication |
| FreeRTOS | Task-based embedded processing |
| Wi-Fi TCP | ESP32-to-Qt communication |
| JSON | Structured sensor data |
| Qt/QML | Desktop graphical interface |
| Qt Network | TCP client communication |
| HTTPS | Remote notification requests |
| ntfy | Notification delivery |
| KiCad | Electronic schematic and PCB design |

---

## Hardware Connections

### MAX30102

| MAX30102 | ESP32 DevKit V1 |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| GND | GND |
| VIN | Compatible supply for the sensor breakout |

### MPU6050

| MPU6050 | ESP32 DevKit V1 |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| GND | GND |
| VCC | Supply supported by the breakout board |

Both sensors share the I²C bus.

The firmware uses the following expected I²C addresses:

- MAX30102: `0x57`
- MPU6050: `0x68` or `0x69`

### Additional Indicators

The following GPIO assignments are used for the optional hardware additions:

| Component | ESP32 GPIO |
|---|---|
| Active buzzer | GPIO 25 |
| Battery-low LED | GPIO 26 |
| Vital-state LED | GPIO 27 |
| Battery-voltage sensing | GPIO 34 |

LEDs require appropriate current-limiting resistors. A buzzer that exceeds the safe GPIO current capability must use a suitable driver.

Battery-voltage sensing requires a correctly dimensioned voltage divider and appropriate ADC calibration.

### Battery Power Arrangement

The intended power path is:

```text
3.7 V Rechargeable Battery
            |
            v
       TP4056 Module
      B+ / B- terminals
            |
            v
      OUT+ / OUT-
            |
            v
     MT3608 Boost Converter
        IN+ / IN-
            |
            v
       Regulated 5 V
        OUT+ / OUT-
            |
            v
      ESP32 5V/VIN and GND
```

The MT3608 output must be measured and verified before connection to the ESP32.

The TP4056 board must be checked for battery protection and a suitable charging-current setting. The charger should not be assumed to provide regulated 5 V or automatic power-path management.

**Current power limitation:** The prototype has shut down during battery-powered operation after approximately two minutes, while USB-powered operation resumed normally. The cause has not yet been confirmed. Battery voltage, converter output under load, current demand, and protection behavior require investigation.

---

## Monitoring Workflow

### 1. Initialization

The ESP32 initializes the I²C bus and sensors, loads stored posture calibration values, and attempts to connect to Wi-Fi.

### 2. Physiological Measurement

The MAX30102 supplies optical pulse data used to estimate heart rate and SpO₂.

Measurement-validity flags help distinguish usable readings from invalid or unavailable measurements.

### 3. Posture Calibration

The MPU6050 provides acceleration data for calibration-based posture classification.

The firmware supports posture-calibration commands:

- `CAL_STAND`
- `CAL_SIT`
- `CAL_LIE`

Classification quality depends on sensor placement, calibration, and body movement.

### 4. Resting Baseline

The monitoring state machine begins in `WAIT_REST`.

A configured 30-second resting period is used to collect valid heart-rate samples. At least ten valid samples are required for a baseline to be accepted.

### 5. Standing Observation

After a valid baseline is obtained, the system enters `READY`.

When standing is detected, the system enters `TESTING` and starts a three-minute observation period.

The heart-rate difference is calculated as:

```text
Heart Rate Difference =
Current Heart Rate - Resting Baseline Heart Rate
```

The firmware uses a sustained increase of at least 30 BPM as its configured alert criterion.

This is an engineering threshold for the prototype and does not independently diagnose a medical condition.

### 6. Event Reporting

Dizziness can be reported using the supported button, serial, or Qt input mechanisms.

When configured and connected, the firmware can send event notifications through ntfy.

---

## Communication Protocol

The ESP32 provides a TCP server on port `8888` and transmits newline-delimited JSON messages approximately every 500 milliseconds.

The Qt application receives and parses these messages.

Example message structure:

```json
{
  "type": "vitals",
  "hr_bpm": 75.0,
  "spo2_pct": 98,
  "hr_valid": true,
  "spo2_valid": true,
  "posture": "SITTING",
  "mpu_ok": true,
  "max_ok": true,
  "ortho_state": "BASELINE",
  "ortho_flag": false
}
```

The values above are illustrative, not results from a recorded experiment. The actual JSON fields depend on the firmware version.

### Supported Commands

| Command | Function |
|---|---|
| `DIZZY` | Reports a dizziness event |
| `CAL_STAND` | Calibrates standing posture |
| `CAL_SIT` | Calibrates sitting posture |
| `CAL_LIE` | Calibrates lying posture |
| `RESET` | Resets the monitoring test |
| `SCAN` | Requests an I²C scan |
| `TEST_NOTIFY` | Tests notification delivery |

---

## Qt Dashboard

The Qt/QML dashboard provides a desktop interface for monitoring the system.

Its intended functions include:

- Displaying heart rate and SpO₂ estimates.
- Showing current posture.
- Displaying sensor initialization and validity status.
- Presenting the monitoring state and elapsed time.
- Showing the resting heart-rate baseline and current difference.
- Reporting dizziness events.
- Sending supported commands to the ESP32.

Add a screenshot of your actual Qt interface here.

```markdown
![Qt Monitoring Dashboard](images/qt-dashboard.png)
```

Replace the image path with your actual screenshot filename.

---

## KiCad Hardware Design

The repository's hardware section is intended to document the electronic design and its relationship to the assembled prototype.

Suggested contents:

```text
hardware/
├── kicad/
│   ├── project.kicad_pro
│   ├── project.kicad_sch
│   ├── project.kicad_pcb
│   └── README.md
├── wiring/
│   └── connections.md
└── BOM.md
```

### KiCad Deliverables

- **Schematic:** Electrical connections between the controller, sensors, power circuitry, and indicators.
- **PCB layout:** Component placement and routing, if a PCB has been designed.
- **Bill of Materials:** Component names, quantities, and relevant specifications.
- **Wiring documentation:** Connections used in the assembled prototype.
- **Design images:** Exported schematic or PCB screenshots for quick viewing on GitHub.

Include only the KiCad files that actually exist. If the prototype uses development boards connected by wires and no PCB has been designed, document the wiring diagram rather than claiming that a custom PCB is complete.

---

## Setup Instructions

### ESP32 Firmware

1. Install Arduino IDE and the ESP32 board support package.
2. Install the required MAX3010x library and any other dependencies used by the firmware.
3. Open the actual ESP32 `.ino` project.
4. Select the appropriate ESP32 DevKit V1 board configuration.
5. Configure Wi-Fi credentials and notification settings.
6. Connect the ESP32 over USB and upload the firmware.
7. Open Serial Monitor at `115200` baud.
8. Verify sensor initialization and calibration messages.

### Qt Application

1. Install a compatible Qt version and compiler toolchain.
2. Open the existing Qt project.
3. Configure the correct build kit.
4. Build and run the application.
5. Connect the application to the ESP32's configured IP address and TCP port.
6. Verify JSON reception and dashboard updates.

The exact Qt build instructions depend on the actual project files and Qt version.

### Posture Calibration

With the device mounted in its intended position, perform the calibration commands supported by the firmware.

Check each posture against the actual sensor placement and observe whether the classification remains stable during ordinary movement.

### Notification Configuration

Configure the ntfy topic and any required network settings in the firmware. Test notifications only with a valid configuration and internet connection.

Do not commit Wi-Fi passwords, private notification topics, tokens, or other secrets to GitHub.

---

## Testing and Validation

The following tests are used to evaluate the prototype:

| Test | Purpose |
|---|---|
| I²C detection | Verify sensor presence and communication |
| Heart-rate monitoring | Observe measurement availability and stability |
| SpO₂ estimation | Verify algorithm output and validity flags |
| Posture calibration | Check standing, sitting, and lying classifications |
| Baseline acquisition | Verify the resting interval and valid-sample requirements |
| Standing test | Verify test timing and state transitions |
| Alert processing | Check configured heart-rate response conditions |
| TCP communication | Verify JSON delivery to Qt |
| Event reporting | Test dizziness commands from supported inputs |
| Notifications | Verify ntfy delivery |
| Battery operation | Measure voltage stability and investigate shutdown |

Numerical accuracy and runtime should be reported only after they have been measured experimentally.

---

## Current Project Status

| Subsystem | Status |
|---|---|
| ESP32 firmware | Under development |
| MAX30102 integration | Implemented; measurement validation required |
| MPU6050 integration | Implemented; posture validation required |
| Resting-baseline state machine | Implemented in firmware |
| Standing observation logic | Implemented in firmware |
| Wi-Fi TCP/JSON communication | Implemented in firmware |
| Qt dashboard | Developed; verify against the current firmware protocol |
| Dizziness-event handling | Implemented in firmware; verify each input path |
| ntfy notifications | Integrated; delivery depends on configuration and connectivity |
| Battery power system | Requires further validation |
| KiCad documentation | Include available schematic and PCB design files |

---

## Limitations and Safety

- This is an educational and engineering prototype, not a clinically validated medical device.
- The system does not directly measure blood pressure and cannot diagnose orthostatic hypotension.
- Heart-rate and SpO₂ estimates may be affected by movement, sensor contact, perfusion, and signal quality.
- Posture classification depends on calibration and sensor placement.
- The baseline and alert thresholds are configured engineering parameters, not a substitute for clinical evaluation.
- Battery-powered operation requires additional electrical validation.
- The system should not be relied upon to prevent falls or provide emergency medical protection.

---

## Future Improvements

- Improve posture classification and transition detection.
- Add data logging and historical trend visualization.
- Improve Qt dashboard reliability and connection recovery.
- Add battery-voltage monitoring and runtime estimation.
- Validate converter stability and charging behavior.
- Improve sensor signal-quality checks.
- Conduct controlled comparisons against suitable reference devices.
- Complete and document the KiCad schematic and PCB design, if applicable.

---

## Technologies

`ESP32` · `C/C++` · `Arduino` · `MAX30102` · `MPU6050` · `I²C` · `FreeRTOS` · `Wi-Fi` · `TCP/IP` · `JSON` · `Qt` · `QML` · `KiCad` · `ntfy`

## License

Choose a license appropriate for your intended use and include the corresponding license file in the repository.

## Author

**[Your Name]**

Embedded Systems | Biomedical Monitoring | IoT | Hardware Design

GitHub: [Your GitHub Profile](https://github.com/)

Replace the author information and profile link before publishing.
