# Docker Setup for ROS2 Workspace on Raspberry Pi 5

## Prerequisites

- Docker installed on Raspberry Pi 5
- This workspace copied to Pi

## Quick Start

### Method 1: Using Docker Compose (Recommended)

```bash
# Build the Docker image
docker-compose build

# Run the container
docker-compose up -d

# Access the container
docker-compose exec ros2 bash

# Inside container - test your packages
ros2 pkg list | grep -E "my_bot|serial_motor|xv11"

# Run your launch file
ros2 launch xv11_lidar_python xv11_lidar.launch.py
```

### Method 2: Using Docker Commands

```bash
# Build the image
docker build -t dev_ws:humble .

# Run the container with device access
docker run -it --rm \
  --privileged \
  --network host \
  -v $(pwd)/src:/workspace/src \
  -v /dev:/dev \
  --device=/dev/ttyUSB0 \
  --device=/dev/ttyACM0 \
  dev_ws:humble

# Inside container
ros2 launch xv11_lidar_python xv11_lidar.launch.py
```

### Method 3: Quick Interactive Session

```bash
# Use pre-built ROS2 image without building
docker run -it --rm \
  --privileged \
  --network host \
  -v $(pwd):/workspace \
  -w /workspace \
  -v /dev:/dev \
  --device=/dev/ttyUSB0 \
  ros:humble bash

# Inside container - build workspace
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# Run your nodes
ros2 launch xv11_lidar_python xv11_lidar.launch.py
```

## Common Commands

### Check Serial Devices

```bash
# On Raspberry Pi (outside container)
ls -l /dev/ttyUSB* /dev/ttyACM*

# Find XV11 lidar device
dmesg | grep tty
```

### Rebuild Workspace Inside Container

```bash
docker-compose exec ros2 bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

### View Logs

```bash
docker-compose logs -f
```

### Stop Container

```bash
docker-compose down
```

## Troubleshooting

### Serial Device Not Found

Update `docker-compose.yml` devices section with correct device path:

```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0 # Your actual device
```

### Permission Denied on Serial Port

```bash
# On host (Raspberry Pi)
sudo chmod 666 /dev/ttyUSB0  # Or your device
```

### Container Won't Start

```bash
# Check logs
docker-compose logs

# Rebuild image
docker-compose build --no-cache
```

## Notes

- The workspace is mounted as a volume, so changes to `src/` persist
- Build artifacts (`build/`, `install/`, `log/`) are inside the container
- Use `--privileged` for full device access
- `network_mode: host` allows ROS2 communication on host network
