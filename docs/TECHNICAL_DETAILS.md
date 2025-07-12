# Technical Details

## Architecture Overview

This project implements a micro-ROS system with the following components:

### 1. ESP32 Firmware Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 Firmware                          │
├─────────────────────────────────────────────────────────────┤
│  Arduino Framework                                         │
│  ├── Setup() - Initialize serial, micro-ROS components     │
│  └── Loop() - Process executor, handle ROS communications  │
├─────────────────────────────────────────────────────────────┤
│  micro-ROS Client Library (rclc)                          │
│  ├── Node Management                                       │
│  ├── Publisher/Subscriber                                  │
│  ├── Timer Management                                      │
│  └── Executor                                             │
├─────────────────────────────────────────────────────────────┤
│  micro-ROS Middleware (rmw_microxrcedds)                  │
│  ├── DDS-XRCE Protocol                                    │
│  ├── Serialization (CDR)                                  │
│  └── Transport Layer                                      │
├─────────────────────────────────────────────────────────────┤
│  Serial Transport                                          │
│  └── UART Communication (115200 baud)                     │
└─────────────────────────────────────────────────────────────┘
```

### 2. Host PC Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Host PC (Ubuntu)                        │
├─────────────────────────────────────────────────────────────┤
│  ROS2 Humble Ecosystem                                     │
│  ├── ros2 topic echo                                       │
│  ├── ros2 node list                                        │
│  └── Other ROS2 tools                                     │
├─────────────────────────────────────────────────────────────┤
│  micro-ROS Agent                                          │
│  ├── Serial Communication                                  │
│  ├── DDS-XRCE Server                                      │
│  ├── Message Translation                                   │
│  └── ROS2 Integration                                     │
├─────────────────────────────────────────────────────────────┤
│  Serial Transport                                          │
│  └── USB Serial (/dev/ttyUSB0)                           │
└─────────────────────────────────────────────────────────────┘
```

## Communication Flow

### Message Publishing Flow

1. **Timer Callback**: ESP32 timer triggers every 1000ms
2. **Message Update**: Increment integer value in message
3. **Publish**: Call `rcl_publish()` to publish message
4. **Serialization**: Message serialized using CDR
5. **Transport**: Serialized data sent via UART
6. **Agent Reception**: micro-ROS agent receives serial data
7. **Translation**: Agent translates to ROS2 message
8. **ROS2 Publication**: Message published to ROS2 topic

### Connection Establishment

```
ESP32                           Host PC
  |                               |
  |-- Serial Connection --------->|
  |                               |-- Agent Start
  |<-- Client Creation Request ---|
  |                               |
  |-- Session Establishment ----->|
  |                               |-- Session Created
  |<-- Participant Creation ------|
  |                               |
  |-- Topic Creation ------------>|
  |                               |-- Topic Registered
  |<-- Publisher Creation --------|
  |                               |
  |-- DataWriter Creation ------->|
  |                               |-- Ready for Messages
  |<-- Publishing Ready ----------|
```

## Message Types and Serialization

### Standard Messages Used

- **std_msgs/Int32**: 32-bit signed integer
- **Serialization**: Common Data Representation (CDR) format
- **Transport**: DDS-XRCE protocol over serial

### Memory Layout

```cpp
typedef struct std_msgs__msg__Int32 {
    int32_t data;
} std_msgs__msg__Int32;
```

## Build System Details

### PlatformIO Configuration

```ini
[env:esp32doit-devkit-v1]
platform = espressif32        # ESP32 platform
board = esp32dev              # Board type
framework = arduino           # Arduino framework
monitor_speed = 115200        # Serial monitor speed
lib_deps = 
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = humble # ROS2 distribution
```

### Compilation Process

1. **Dependency Resolution**: PlatformIO downloads micro-ROS library
2. **Python Environment**: Creates isolated Python environment
3. **micro-ROS Build**: Compiles micro-ROS libraries for ESP32
4. **Firmware Compilation**: Compiles main application
5. **Linking**: Links all components into final firmware

## Performance Characteristics

### Memory Usage

- **RAM**: ~41KB (12.6% of 327KB available)
- **Flash**: ~328KB (25% of 1.3MB available)
- **Stack**: Dynamic allocation for ROS operations

### Timing Characteristics

- **Publishing Rate**: 1 Hz (configurable)
- **Serial Baud Rate**: 115200 bps
- **Message Latency**: ~10-50ms (depends on system load)

## Error Handling

### ESP32 Error Handling

```cpp
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        error_loop(); \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        /* Continue execution */ \
    } \
}
```

### Common Error Conditions

1. **Memory Allocation Failures**: Handled by error_loop()
2. **Communication Timeouts**: Soft errors, continue execution
3. **Serialization Errors**: Logged and skipped
4. **Hardware Failures**: System reset required

## Development Environment

### Required Tools

- **PlatformIO Core**: Build system for embedded development
- **ESP32 Toolchain**: Cross-compilation tools
- **Python Dependencies**: Code generation and build tools
- **ROS2 Humble**: Host-side ROS2 environment

### Debug Configuration

```bash
# Enable verbose PlatformIO output
pio run -v

# Monitor serial output
pio device monitor --baud 115200

# Check micro-ROS agent logs
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

## Customization Points

### Adding New Message Types

1. **Include Header**: Add message type header
2. **Declare Variable**: Create message instance
3. **Initialize Publisher**: Update publisher initialization
4. **Update Callback**: Modify timer callback logic

### Adding Subscribers

```cpp
// Add subscriber declaration
rcl_subscription_t subscriber;
std_msgs__msg__Int32 sub_msg;

// Initialize subscriber
RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "input_topic"));

// Add to executor
RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &subscriber, 
    &sub_msg, 
    &subscription_callback, 
    ON_NEW_DATA));
```

### Transport Layer Alternatives

micro-ROS supports multiple transport layers:

- **Serial**: Current implementation
- **WiFi/UDP**: For wireless communication
- **Ethernet**: For wired network communication
- **CAN**: For automotive applications

## Security Considerations

### Current Security Model

- **No Authentication**: Serial communication is unencrypted
- **Physical Security**: USB connection required
- **Local Network**: Agent runs on localhost

### Security Enhancements

For production use, consider:

1. **TLS/SSL**: Encrypt communications
2. **Authentication**: Validate client identity
3. **Authorization**: Control topic access
4. **Firewall**: Network-level protection

## Testing and Validation

### Unit Tests

Currently, the project focuses on integration testing:

1. **Build Test**: Firmware compiles successfully
2. **Connection Test**: Agent establishes connection
3. **Message Test**: Messages published correctly
4. **Timing Test**: Publishing rate is consistent

### Integration Tests

```bash
# Test complete pipeline
./test_integration.sh

# Test serial communication
./test_serial.sh

# Test message publishing
./test_publishing.sh
```

## Future Enhancements

### Planned Features

1. **Multiple Topics**: Support for multiple publishers/subscribers
2. **Services**: Request/response communication
3. **Parameters**: Dynamic configuration
4. **Actions**: Long-running operations
5. **Sensor Integration**: I2C/SPI sensor support

### Performance Optimizations

1. **Buffer Management**: Optimize memory usage
2. **Interrupt Handling**: Reduce latency
3. **Power Management**: Sleep modes for battery operation
4. **Compression**: Reduce bandwidth usage 