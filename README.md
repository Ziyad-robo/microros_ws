# micro-ROS ESP32 Project

A complete micro-ROS implementation for ESP32 that demonstrates basic ROS2 communication between an ESP32 microcontroller and a host PC running ROS2 Humble.

## Overview

This project implements a micro-ROS node on an ESP32 that publishes integer messages to a ROS2 topic. The ESP32 acts as a micro-ROS client that communicates with a micro-ROS agent running on the host PC via serial communication.

**Features:**
- ESP32 running micro-ROS firmware with PlatformIO
- Serial communication transport between ESP32 and host PC
- Integer message publishing with auto-incrementing values
- Complete build system with dependency management
- Compatible with ROS2 Humble

## Hardware Requirements

- ESP32 Dev Kit V1 (or compatible ESP32 board)
- Micro USB cable
- Host PC with Ubuntu 22.04 (native or VM)

## Software Requirements

- **Host PC:**
  - Ubuntu 22.04 LTS
  - ROS2 Humble
  - PlatformIO Core
  - Python 3.10+

- **ESP32:**
  - Arduino framework
  - micro-ROS PlatformIO library

## Quick Start

### 1. Setup Host PC Environment

```bash
# Install ROS2 Humble (follow official documentation)
source /opt/ros/humble/setup.bash

# Option A: Clone this repository
git clone <your-repo-url>
cd microros_ws

# Option B: Create workspace from scratch
mkdir -p ~/microros_ws/src
cd ~/microros_ws
```

### 2. Install micro-ROS Setup Package

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Clone micro-ROS setup package
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools
colcon build
source install/local_setup.bash
```

### 3. Create micro-ROS Agent Workspace

```bash
# Create agent workspace (this will clone micro-ROS agent packages into src/uros/)
ros2 run micro_ros_setup create_agent_ws.sh

# Build the agent
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

**Note:** The `create_agent_ws.sh` script will create the `src/uros/` directory and clone the following packages:
- `micro_ros_msgs` - Standard micro-ROS message definitions
- `micro-ROS-Agent` - The micro-ROS agent that bridges micro-ROS clients to ROS2

### 3.1 Create Firmware Project (if starting from scratch)

If you're creating the firmware project from scratch, follow these steps:

```bash
# Create firmware directory
mkdir -p src/firmware/src

# Create PlatformIO configuration
cat > src/firmware/platformio.ini << 'EOF'
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = 
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = humble
EOF

# Copy the main.cpp file from the repository or create your own
# (See src/firmware/src/main.cpp in this repository)
```

### 4. Build and Upload ESP32 Firmware

```bash
cd src/firmware

# Install dependencies (first time only)
chmod +x setup_dependencies.sh
./setup_dependencies.sh

# Build firmware
pio run

# Upload to ESP32 (connect via USB first)
pio run --target upload
```

### 5. Run the System

**Terminal 1 - Start micro-ROS Agent:**
```bash
cd microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

**Terminal 2 - Monitor Messages:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /micro_ros_platformio_node_publisher
```

## Project Structure

```
microros_ws/
├── src/
│   ├── firmware/                 # ESP32 firmware
│   │   ├── src/
│   │   │   └── main.cpp         # Main ESP32 application
│   │   ├── platformio.ini       # PlatformIO configuration
│   │   ├── requirements.txt     # Python dependencies
│   │   └── setup_dependencies.sh # Dependency installation script
│   ├── micro_ros_setup/         # micro-ROS setup tools
│   └── uros/                    # micro-ROS packages
├── install/                     # Built packages
├── build/                       # Build artifacts
├── log/                         # Build logs
└── README.md                    # This file
```

## How It Works

1. **ESP32 Firmware**: Runs a micro-ROS node that publishes `std_msgs/Int32` messages
2. **Serial Transport**: Communication between ESP32 and host PC via USB serial
3. **micro-ROS Agent**: Bridges micro-ROS messages to full ROS2 ecosystem
4. **Message Flow**: ESP32 → Serial → micro-ROS Agent → ROS2 Topics

## Customization

### Modify Message Type

Edit `src/firmware/src/main.cpp` to change the message type:

```cpp
// Change from Int32 to String
#include <std_msgs/msg/string.h>
std_msgs__msg__String msg;
```

### Change Topic Name

```cpp
// Modify the topic name in the publisher initialization
RCCHECK(rclc_publisher_init_default(
  &publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
  "your_custom_topic_name"));
```

### Adjust Publishing Rate

```cpp
// Change timer period (in milliseconds)
const unsigned int timer_timeout = 2000; // 2 seconds
```

## Troubleshooting

### Common Issues

1. **Permission Denied on /dev/ttyUSB0**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # Or add user to dialout group
   sudo usermod -a -G dialout $USER
   ```

2. **MD5 Mismatch During Upload**
   ```bash
   pio run --target erase
   pio run --target upload
   ```

3. **Missing Python Dependencies**
   ```bash
   cd src/firmware
   ./setup_dependencies.sh
   ```

4. **Agent Connection Issues**
   - Check ESP32 is connected and powered
   - Verify correct serial port (`ls /dev/tty*`)
   - Ensure ESP32 firmware is running

### Debug Serial Output

```bash
cd src/firmware
pio device monitor --baud 115200
```

## Expected Output

When everything is working correctly:

**micro-ROS Agent Output:**
```
[1752321242.425843] info     | TermiosAgentLinux.cpp | init                     | running...             | fd: 3
[1752321313.973417] info     | Root.cpp           | create_client            | create                 | client_key: 0x36FE6047, session_id: 0x81
[1752321314.039894] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x36FE6047, participant_id: 0x000(1)
[1752321314.057896] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x36FE6047, topic_id: 0x000(2), participant_id: 0x000(1)
[1752321314.068601] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x36FE6047, publisher_id: 0x000(3), participant_id: 0x000(1)
[1752321314.080953] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x36FE6047, datawriter_id: 0x000(5), publisher_id: 0x000(3)
```

**ROS2 Topic Output:**
```
data: 20
---
data: 21
---
data: 22
---
```

## References

- [micro-ROS ESP32 Tutorial](https://technologiehub.at/project-posts/micro-ros-on-esp32-tutorial/)
- [micro-ROS Official Documentation](https://micro.ros.org/)
- [PlatformIO ESP32 Platform](https://docs.platformio.org/en/latest/platforms/espressif32.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## License

This project is released under the MIT License. See LICENSE file for details.



## Acknowledgments

- Based on the excellent tutorial by [Technologie Hub Wien](https://technologiehub.at/project-posts/micro-ros-on-esp32-tutorial/)
- micro-ROS development team
- ESP32 Arduino framework community 