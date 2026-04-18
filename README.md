# Diff-Drive Robot — STM32 + Raspberry Pi + ROS2

A ROS2-compatible differential drive robot. The STM32F407 handles all real-time hardware tasks (motor control, encoder reading, IMU, LED, buzzer). A Raspberry Pi 3B+ acts as a **transparent serial–FastDDS bridge** (no packet decoding). The Laptop running ROS2 Ubuntu performs all packet encoding/decoding and exposes standard ROS2 topics.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Laptop (Ubuntu + ROS2)                   │
│                                                                 │
│   ROS2 Topics  ◄──►  ros2_bridge node  ◄──►  FastDDS (raw)     │
│  /wheel_enc         (encode / decode)        ByteSeq topics     │
│  /cmd_vel                                                       │
│  /imu_data                                                      │
│  /led_ctrl   ...                                                │
└───────────────────────────────┬─────────────────────────────────┘
								│  FastDDS over LAN/WiFi
								│  (raw byte-sequence topics)
┌───────────────────────────────▼─────────────────────────────────┐
│                  Raspberry Pi 3 Model B+ (pi_bridge)            │
│                                                                 │
│   FastDDS (raw) ◄──────────── forward ──────────────► UART     │
│                                                                 │
│   Pi does NOT decode packets.                                   │
│   It only forwards raw bytes between UART and FastDDS.          │
└───────────────────────────────┬─────────────────────────────────┘
								│  UART (binary packets)
								│  STX | Data | CRC | ETX
┌───────────────────────────────▼─────────────────────────────────┐
│                       STM32F407 (MCU)                           │
│                                                                 │
│  • Motor driver (diff-drive left/right)                         │
│  • Wheel encoder reading                                        │
│  • PID speed controller                                         │
│  • IMU data reading                                             │
│  • LED and buzzer control                                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## Data Flow

### Uplink — Sensor data (MCU → Laptop)

```
STM32  →[encode packet]→  UART  →  Pi (forward raw bytes)  →  FastDDS  →  Laptop  →[decode]→  ROS2 topics
```

| Packet type | ROS2 Topic (target) | Direction |
|---|---|---|
| `WHEEL_ENC_COMMAND` | `/wheel_encoder_ticks` | MCU → Laptop |
| `MOTOR_RPM_COMMAND` | `/motor_rpm_feedback` | MCU → Laptop |
| `DEBUG_STRING` | stdout / `/rosout` | MCU → Laptop |
| `IMU_DATA` *(planned)* | `/imu/data` | MCU → Laptop |

### Downlink — Control commands (Laptop → MCU)

```
ROS2 topics  →  Laptop  →[encode packet]→  FastDDS  →  Pi (forward raw bytes)  →  UART  →  STM32  →[decode]→  actuators
```

| ROS2 Topic | Packet type | Direction |
|---|---|---|
| `/cmd_vel` | `CMD_VEL_COMMAND` | Laptop → MCU |
| `/pid_config` | `PID_CONFIG_COMMAND` | Laptop → MCU |
| `/led_control` | `LED_CONTROL_COMMAND` | Laptop → MCU |
| `/buzzer_control` | `BUZZER_CONTROL_COMMAND` | Laptop → MCU |
| `/comm_ctrl` | `COMM_CTRL_COMMAND` | Laptop → MCU |

### Derived Odometry (Laptop ROS2)

`wheel_odometry_node` converts cumulative wheel encoder ticks into:

- `/odom` (`nav_msgs/msg/Odometry`)
- `odom -> base_link` TF (optional)

The node subscribes to `/wheel_encoder_ticks` and uses diff-drive kinematics:

$$
d_L = \Delta ticks_L \cdot \frac{2\pi r}{N},\quad
d_R = \Delta ticks_R \cdot \frac{2\pi r}{N}
$$

$$
d = \frac{d_L + d_R}{2},\quad
\Delta\theta = \frac{d_R - d_L}{L}
$$

where $r$ is wheel radius, $L$ is wheel separation, and $N$ is ticks per revolution.

---

## Packet Protocol

All UART communication uses a framed binary protocol (defined in `library/serial_comm/`):

```
[ STX (0xAA) | payload bytes | CRC (XOR) | ETX (0xDD) ]
```

- Special bytes inside the payload are escaped to avoid collision with frame markers.
- CRC is a simple XOR checksum over payload bytes.
- First byte of payload is always the **command type** byte (see `velocity_control.h`).

### Command Type Codes

| Value | Name | Description |
|---|---|---|
| 0 | `DEBUG_STRING` | Plain-text debug message from MCU |
| 1 | `CMD_VEL_COMMAND` | Left / right wheel RPM setpoint |
| 2 | `WHEEL_ENC_COMMAND` | Left / right encoder tick counts |
| 3 | `PID_CONFIG_COMMAND` | PID gains (Kp, Ki, Kd) |
| 4 | `COMM_CTRL_COMMAND` | Enable/disable feedback streams |
| 5 | `LED_CONTROL_COMMAND` | LED type/mode, RGB color, and effect parameters |
| 6 | `BUZZER_CONTROL_COMMAND` | Buzzer type/mode and effect parameters |
| 7 | `MOTOR_RPM_COMMAND` | Actual motor RPM feedback |

---

## Repository Structure

```
velocity_control/
├── library/                    # Shared libraries (used by both pi_bridge and ros2_bridge)
│   ├── serial_comm/            # Packet protocol: encode, decode, CRC, framing
│   │   ├── velocity_control.h  # Command type definitions and packet structs
│   │   ├── process_data_packet.h/.c  # Encode / decode / CRC functions
│   ├── serial_driver/          # Linux UART serial driver (wraps termios)
│   │   ├── serial_linux.hpp/.cpp
│   │   └── rules/              # udev rules for persistent Arduino/STM32 port naming
│   └── fastdds_comm/           # FastDDS publisher / subscriber wrappers
│
├── pi_bridge/                  # [Runs on Raspberry Pi 3B+]
│   │                           # Transparent bridge: UART ↔ FastDDS raw bytes
│   └── ...                     # No packet decoding here
│
├── ros2_bridge/                # [Runs on Laptop - Ubuntu ROS2]
│   │                           # Decodes incoming packets → publishes ROS2 topics
│   │                           # Subscribes to ROS2 topics → encodes → sends via FastDDS
│   └── ...
│
├── CMakeLists.txt
├── package.xml
└── README.md
```

> **Naming status:**
> - `velocity_server/` has been renamed to `pi_bridge/`
> - `library/linux_comm_driver/` has been renamed to `library/serial_driver/`
> - `library/serial_comm/` is intentionally kept unchanged

---

## Build & Run

### 1. Clone and set up library dependencies

The `library/` subdirectories are separate git repositories. After cloning this repo, fetch them with [vcstool](https://github.com/dirk-thomas/vcstool):

```zsh
cd ~/ros2_ws/src/velocity_control
vcs import < deps.repos
```

If `vcs` is not installed:

```zsh
# On a machine with ROS2 Humble it is already available.
# Otherwise install manually:
pip install vcstool
```

To update all libraries to latest:

```zsh
vcs pull
```

### Dependencies
- Ubuntu 22.04 + ROS2 Humble (recommended)
- FastDDS / Fast-CDR (`fastrtps`, `fastcdr`)
- CMake ≥ 3.8

### Option A: Build with CMake (quick local build)

Use this when you want to run binaries directly from `build/`.

```bash
cd ~/ros2_ws/src/velocity_control
cmake -S . -B build
cmake --build build -j$(nproc)
```

Run executables directly:

```bash
# Raspberry Pi bridge
./build/pi_bridge /dev/mcu_serial 115200

# Laptop bridge node (works as a normal executable)
./build/ros2_bridge
```

### Option B: Build as a ROS2 package (colcon)

Use this if you want `ros2 run velocity_control ros2_bridge`.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select velocity_control
source install/setup.zsh
```

Run ROS2 node:

```bash
ros2 run velocity_control ros2_bridge

# Wheel tick -> odom converter
ros2 run velocity_control wheel_odometry_node
```

Run with custom odometry parameters:

```bash
ros2 run velocity_control wheel_odometry_node --ros-args \
  -p wheel_radius:=0.0335 \
  -p wheel_separation:=0.252 \
  -p ticks_per_revolution:=1560.0 \
  -p odom_frame:=odom \
  -p base_frame:=base_link \
  -p publish_tf:=true
```

### PS3 Joystick to `/cmd_vel` (Laptop ROS2)

This package includes a ready-to-use launch setup for a PS3 controller:

- Launch file: `joy_stick/ps3_cmd_vel.launch.py`
- Teleop config: `joy_stick/ps3_teleop.yaml`

Install required ROS2 packages (Ubuntu Humble):

```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

Build and run from ROS2 workspace (`zsh`):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh
colcon build --packages-select velocity_control
source install/setup.zsh
ros2 launch velocity_control ps3_cmd_vel.launch.py
```

Verify joystick output:

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

Default control mapping in `ps3_teleop.yaml`:

- Hold `L1` to enable normal speed (`enable_button: 4`)
- Hold `R1` for turbo (`enable_turbo_button: 5`)
- Left stick vertical controls `linear.x`
- Right stick horizontal controls `angular.z`

If your PS3 mapping differs on your Linux driver, run:

```bash
ros2 topic echo /joy
```

Then update axis/button indices in `joy_stick/ps3_teleop.yaml` accordingly.

### Runtime Setup (Pi + Laptop)

1. Start `pi_bridge` on Raspberry Pi (connected to STM32 via UART).
2. Start `ros2_bridge` on Laptop (same LAN as Pi).
3. Confirm data topics:

```bash
ros2 topic list
ros2 topic echo /wheel_encoder_ticks
ros2 topic echo /motor_rpm_feedback
ros2 topic echo /mcu_debug
```

4. Send commands from Laptop:

```bash
# cmd_vel -> CMD_VEL_COMMAND
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}"

# pid_config -> PID_CONFIG_COMMAND
ros2 topic pub -1 /pid_config geometry_msgs/msg/Vector3 \
"{x: 1.0, y: 0.2, z: 0.01}"

# comm_ctrl -> COMM_CTRL_COMMAND (example: FEEDBACK_ALL = 255)
ros2 topic pub -1 /comm_ctrl std_msgs/msg/UInt8 "{data: 255}"

# led_control -> LED_CONTROL_COMMAND, format [led_type, r, g, b, param1, param2]
ros2 topic pub -1 /led_control std_msgs/msg/UInt16MultiArray "{data: [1, 255, 0, 0, 100, 500]}"

# buzzer_control -> BUZZER_CONTROL_COMMAND, format [buzzer_type, param1, param2]
ros2 topic pub -1 /buzzer_control std_msgs/msg/UInt16MultiArray "{data: [1, 300, 200]}"
```

### ROS2 Parameters (`ros2_bridge`)

- `wheel_radius` (default `0.05` m)
- `wheel_separation` (default `0.25` m)

Set parameter at run time:

```bash
ros2 run velocity_control ros2_bridge --ros-args -p wheel_radius:=0.048 -p wheel_separation:=0.24
```

### ROS2 Parameters (`wheel_odometry_node`)

- `wheel_radius` (default `0.0335` m)
- `wheel_separation` (default `0.252` m)
- `ticks_per_revolution` (default `1320.0`)
- `odom_frame` (default `odom`)
- `base_frame` (default `base_link`)
- `publish_tf` (default `true`)

---

## Hardware Notes

- **UART baud rate**: 115200 (default)
- **udev rules**: Install `library/serial_driver/rules/70-arduino.rules` on Pi for persistent device naming (`/dev/mcu_serial` or configure as needed for STM32).
- FastDDS transport: UDP multicast on local network (Pi and Laptop must be on the same subnet).
