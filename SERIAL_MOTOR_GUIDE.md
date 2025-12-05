# Serial Motor Demo - Arduino Mega Setup Guide

This guide explains how to use the `serial_motor_demo` package to send commands to your Arduino Mega from the Raspberry Pi 5 running in Docker.

## Overview

The `serial_motor_demo` package provides:

- **Driver Node**: Interfaces with Arduino over serial, sends motor commands
- **GUI Node**: Simple interface to control motors for testing
- **Message Types**: MotorCommand, MotorVels, EncoderVals

## Prerequisites

### 1. Arduino Code Setup

You need to flash the Arduino with the corresponding code from:
https://github.com/joshnewans/ros_arduino_bridge

This code expects serial commands like:

- `o <pwm1> <pwm2>` - Set raw PWM values (-255 to 255)
- `m <counts1> <counts2>` - Set motor speeds in encoder counts per loop
- `e` - Read encoder values

### 2. Connect Arduino to Raspberry Pi

Connect your Arduino Mega to the Raspberry Pi via USB.

Find the device:

```bash
# On Raspberry Pi (outside Docker)
ls -l /dev/ttyUSB* /dev/ttyACM*
# Arduino Mega typically shows as /dev/ttyACM0
```

### 3. Update Docker Configuration

Edit `docker-compose.yml` to include your Arduino device:

```yaml
devices:
  - /dev/ttyACM0:/dev/ttyACM0 # Arduino Mega
  - /dev/ttyUSB0:/dev/ttyUSB0 # XV11 Lidar (if using)
```

Or give permission:

```bash
sudo chmod 666 /dev/ttyACM0
```

## Usage

### Method 1: Using the GUI (Easiest for Testing)

```bash
# Inside Docker container
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Run the GUI
ros2 run serial_motor_demo gui
```

The GUI provides:

- PWM Mode: Direct motor control (-255 to 255)
- Closed-loop Mode: Speed control in rad/sec
- Real-time feedback display

### Method 2: Using the Driver + Command Line

#### Start the driver:

```bash
# Inside Docker container
ros2 run serial_motor_demo driver --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baud_rate:=57600 \
  -p encoder_cpr:=3440 \
  -p loop_rate:=30
```

**Parameters:**

- `serial_port`: Arduino device path (default: `/dev/ttyUSB0`)
- `baud_rate`: Serial baud rate (default: `57600`)
- `encoder_cpr`: Encoder counts per revolution
- `loop_rate`: Arduino loop rate in Hz
- `serial_debug`: Enable debug output (default: `false`)

#### Send commands in another terminal:

```bash
# Open another terminal and enter Docker
docker exec -it ros2_dev_ws bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Send PWM command (raw motor values -255 to 255)
ros2 topic pub /motor_command serial_motor_demo_msgs/msg/MotorCommand \
  "{is_pwm: true, mot_1_req_rad_sec: 100.0, mot_2_req_rad_sec: 100.0}"

# Send speed command (rad/sec, requires encoders)
ros2 topic pub /motor_command serial_motor_demo_msgs/msg/MotorCommand \
  "{is_pwm: false, mot_1_req_rad_sec: 3.14, mot_2_req_rad_sec: 3.14}"
```

#### Monitor feedback:

```bash
# View motor velocities
ros2 topic echo /motor_vels

# View encoder values
ros2 topic echo /encoder_vals
```

### Method 3: Using Python Script

Create a test script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

    def send_pwm_command(self, motor1_pwm, motor2_pwm):
        msg = MotorCommand()
        msg.is_pwm = True
        msg.mot_1_req_rad_sec = float(motor1_pwm)
        msg.mot_2_req_rad_sec = float(motor2_pwm)
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent PWM: M1={motor1_pwm}, M2={motor2_pwm}')

def main():
    rclpy.init()
    node = MotorTester()

    # Example: Send forward command
    node.send_pwm_command(150, 150)

    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Message Types

### MotorCommand (Published to `/motor_command`)

```
bool is_pwm                  # true = PWM mode, false = speed control
float64 mot_1_req_rad_sec   # Motor 1 value (PWM: -255 to 255, Speed: rad/sec)
float64 mot_2_req_rad_sec   # Motor 2 value (PWM: -255 to 255, Speed: rad/sec)
```

### MotorVels (Subscribed from `/motor_vels`)

```
float64 mot_1_rad_sec       # Motor 1 actual speed (rad/sec)
float64 mot_2_rad_sec       # Motor 2 actual speed (rad/sec)
```

### EncoderVals (Subscribed from `/encoder_vals`)

```
int64 mot_1_enc_val         # Motor 1 encoder count
int64 mot_2_enc_val         # Motor 2 encoder count
```

## Quick Test Procedure

1. **Start Docker container:**

   ```bash
   cd ~/dev_ws
   docker compose up -d
   docker exec -it ros2_dev_ws bash
   ```

2. **Source workspace:**

   ```bash
   source /opt/ros/humble/setup.bash
   source /workspace/install/setup.bash
   ```

3. **Test serial connection:**

   ```bash
   ros2 run serial_motor_demo driver --ros-args \
     -p serial_port:=/dev/ttyACM0 \
     -p serial_debug:=true
   ```

4. **In another terminal, send test command:**

   ```bash
   docker exec -it ros2_dev_ws bash
   source /opt/ros/humble/setup.bash
   source /workspace/install/setup.bash

   # Test motors forward
   ros2 topic pub --once /motor_command serial_motor_demo_msgs/msg/MotorCommand \
     "{is_pwm: true, mot_1_req_rad_sec: 50.0, mot_2_req_rad_sec: 50.0}"
   ```

## Troubleshooting

### Device not found

```bash
# Check device exists
ls -l /dev/ttyACM*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Verify in docker-compose.yml devices section
```

### Serial timeout

- Check baud rate matches Arduino code (default: 57600)
- Verify Arduino is properly flashed with ros_arduino_bridge code
- Enable serial debug: `-p serial_debug:=true`

### No response from Arduino

- Arduino may need time to reset after opening serial port (driver waits 2 seconds)
- Check Arduino serial monitor first to verify it's working
- Try pressing reset button on Arduino after starting driver

### GUI not showing

If running in Docker without display:

- Use command-line method instead
- Or set up X11 forwarding (more complex)

## Integration with Your Robot

Once tested, you can integrate this into your robot control system:

1. Start driver as a background service
2. Your navigation/control nodes publish to `/motor_command`
3. Use `/motor_vels` for odometry feedback
4. Use `/encoder_vals` for precise position tracking

## Example Launch File

Create `~/dev_ws/launch/motor_demo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_motor_demo',
            executable='driver',
            name='motor_driver',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 57600,
                'encoder_cpr': 3440,
                'loop_rate': 30,
                'serial_debug': False
            }]
        )
    ])
```

Run with:

```bash
ros2 launch motor_demo.launch.py
```
