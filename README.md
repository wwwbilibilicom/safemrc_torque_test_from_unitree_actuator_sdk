# SafeMRC & Unitree Motor Control Toolkit

A comprehensive toolkit for real-time control, monitoring, and safety management of Unitree GO-M8010-6, A1, B1 motors, and SafeMRC devices. Includes robust C++ and Python examples, serial communication, automatic zero calibration, collision detection, and a high-performance UI for live data visualization.

---

## Features
- **SafeMRC SDK**: Python interface for SafeMRC device communication (serial, CRC, protocol parsing)
- **Unitree Motor Control**: Real-time control and monitoring for GO-M8010-6, A1, B1 motors (C++ & Python)
- **Automatic Zero Offset Calibration**: Ensures accurate position reference at startup
- **Collision Detection**: Real-time detection with latching and manual reset
- **High-Performance UI**: PyQt5/pyqtgraph-based live plotting and collision status
- **Parameter Conversion Guidance**: Rotor/output side conversion, gear ratio notes
- **English Documentation & Code**: All code, comments, and UI in English for international use

---

## Environment Setup

### C++ Build Requirements
- gcc >= 5.4.0 (x86) or gcc >= 7.5.0 (ARM)
- CMake

#### Build Steps
```bash
mkdir build
cd build
cmake ..
make
```

### Python Environment
It is recommended to use a dedicated conda environment:
```bash
conda create -n safemrc-sdk python=3.9
conda activate safemrc-sdk
conda install pyserial pyqt5 pyqtgraph matplotlib
```

---

## Unitree Motor Examples (C++ & Python)

### Supported Motors
- GO-M8010-6
- A1
- B1

### C++ Example Usage
After building, run C++ examples with sudo:
```bash
sudo ./example_a1_motor
```

### Python Example Usage
Navigate to the `python/` folder and run:
```bash
sudo python3 example_a1_motor.py
```

### Parameter Conversion (Rotor/Output Side)
When assigning values to the command structure `cmd`, note:
- All commands are for the **rotor** side.
- Usual calculations are for the **output** side; conversion is needed.

**Conversion formulas:**
```
kp_rotor = kp_output / r^2
kd_rotor = kd_output / r^2
```
Where `r` is the gear ratio.

For A1/B1, see `example_a1_motor_output.cpp` for additional scaling (magic numbers).

---

## SafeMRC SDK Usage

The SafeMRC SDK enables Python communication with SafeMRC devices via serial port.

### Key Classes
- `SafeMRCCmd`: Command structure (mode, current, id)
- `SafeMRCData`: Feedback structure (id, mode, collision, encoder, velocity, current)
- `SafeMRC`: Main SDK class for serial protocol and communication

### Example: Basic Communication
```python
from safeMRC_sdk import SafeMRC, SafeMRCCmd, SafeMRCData
import time

safe_mrc = SafeMRC('/dev/ttyUSB1')
cmd = SafeMRCCmd(mode=1, current=0.5)
fbk = SafeMRCData()

if safe_mrc.sendRecv(cmd, fbk):
    print(f"Feedback: id={fbk.id}, mode={fbk.mode}, collision={fbk.collision}, "
          f"encoder={fbk.encoder:.5f}, velocity={fbk.velocity:.5f}, current={fbk.current:.5f}")
else:
    print("No valid response or CRC error.")
```

### Example: Continuous Control
```python
for i in range(10):
    cmd.current = 0.1 * i
    if safe_mrc.sendRecv(cmd, fbk):
        print(f"Step {i}: encoder={fbk.encoder:.5f}, velocity={fbk.velocity:.5f}")
    time.sleep(0.1)
```

### SafeMRC Modes

| Mode Value | Name         | Description                                 |
|:----------:|:------------|:--------------------------------------------|
| 0          | FREE        | Free mode (no active control, output off)   |
| 1          | FIX_LIMIT   | Fixed limit mode (position/safety limit)    |
| 2          | ADAPTATION  | Adaptation mode (compliance, soft control)  |
| 3          | DEBUG       | Debug mode (for development/testing)        |

### SafeMRC Communication Protocol

#### Command Frame Structure
| Field         | Type      | Bytes | Description                        |
|:-------------|:----------|:-----:|:-----------------------------------|
| Header       | uint8[2]  | 2     | 0xFE, 0xEE (frame header)          |
| ID           | uint8     | 1     | Device ID                          |
| Mode         | uint8     | 1     | Control mode (see table above)     |
| Current      | int32     | 4     | Desired coil current (mA, little-endian) |
| CRC16        | uint16    | 2     | CRC-CCITT checksum (little-endian) |
| **Total**    |           | **10**|                                   |

#### Feedback Frame Structure
| Field         | Type      | Bytes | Description                        |
|:-------------|:----------|:-----:|:-----------------------------------|
| Header       | uint8[2]  | 2     | 0xFE, 0xEE (frame header)          |
| ID           | uint8     | 1     | Device ID                          |
| Mode         | uint8     | 1     | Current mode                       |
| Collision    | uint8     | 1     | Collision flag (0: safe, 1: collision) |
| Encoder      | int32     | 4     | Encoder value (signed, little-endian) |
| Velocity     | int32     | 4     | Encoder velocity (signed, little-endian) |
| Current      | int16     | 2     | Present current (signed, little-endian, mA) |
| CRC16        | uint16    | 2     | CRC-CCITT checksum (little-endian) |
| **Total**    |           | **17**|                                   |

**Note:** All multi-byte fields use little-endian byte order. CRC is calculated over all bytes except the CRC field itself.

---

## Motor Collision Detection UI

`goM8010_6_motor_collision_detection.py` provides:
- Real-time serial communication with Unitree GO-M8010-6 motor
- Automatic zero offset calibration (1000 samples at startup)
- Real-time collision detection (bandpass filter, latching, manual reset)
- Data buffering and time-stamped logging
- PyQt5/pyqtgraph UI for live plotting of position and velocity
- Collision indicator and manual reset button

### How to Run
```bash
python3 python/goM8010_6_motor_collision_detection.py
```

### UI Features
- **Live Plots**: Position and velocity, configurable time window
- **Collision Indicator**: Red when collision detected, manual reset required
- **Zero Offset Calibration**: Performed at startup, keep motor still
- **All UI and code in English**

---

## Example Outputs

**SafeMRC SDK:**
```
id=1 | mode=0 | collision=0 | encoder=0.00012 | velocity=-0.05700 | current=-0.00700
id=1 | mode=0 | collision=0 | encoder=0.00029 | velocity=0.02800 | current=0.00100
...
```

**Motor Collision Detection:**
```
motor 0: cmd.q=   0.000 | q=   0.00000 | dq=   0.00000 | collision=False | collision_count=  0
motor 0: cmd.q=   0.000 | q=   0.00001 | dq=   0.00002 | collision=False | collision_count=  0
...
```

---

## Notes & Troubleshooting
- The SDK and UI are robust to noise and designed for real-time operation.
- If you see "No valid response or CRC error.", check your hardware connection and CRC table consistency.
- For advanced usage, refer to the source code for protocol details and error handling.
- All documentation and code are in English for international collaboration.

---

## Contact & Support
If you have any questions or need assistance, please contact support@unitree.com or open an issue on GitHub.
