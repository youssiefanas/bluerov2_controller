# bluerov2_controller

C++ ROS 2 Jazzy package for controlling a BlueROV2 with BlueOS. Provides joystick-based manual control, sensor data processing, and low-latency camera streaming.

## Architecture

```
                    BlueROV2 (192.168.2.2)
                    ├── Pixhawk FC (MAVLink UDP:14550)
                    └── H.264 Camera (UDP:5600)
                            │
                ┌───────────┴───────────┐
                │   Ethernet Tether     │
                └───────────┬───────────┘
                            │
            Host Computer (192.168.2.1) - ROS 2
            ┌───────────────┴───────────────┐
            │                               │
     ┌──────┴──────┐                ┌───────┴───────┐
     │   MAVROS    │                │ camera_       │
     │   Bridge    │                │ streamer      │
     └──────┬──────┘                │ (GStreamer)   │
            │                       └───────┬───────┘
    /imu/data, /rel_alt,                    │
    /cmd/command, /rc/override      /camera/image
            │                               │
     ┌──────┴──────────────────┐            │
     │    rov_controller       │            │
     │  (joystick + control)   │            │
     └──────┬──────────────────┘            │
            │                               │
     ┌──────┴──────┐                        │
     │  joy_node + │                        │
     │ teleop_     │                        │
     │ twist_joy   │                        │
     └─────────────┘                        │
      Xbox Controller              rqt_image_view
```

## Nodes

### rov_controller

Replaces the Python `listenerMIR` node. Handles joystick input, motor control via MAVROS RC override, IMU processing, and servo control for lights and camera tilt.

| Published Topics | Type | Description |
|---|---|---|
| `rc/override` | mavros_msgs/OverrideRCIn | 8-channel motor PWM (20 Hz) |
| `angle_degree` | geometry_msgs/Twist | Roll/pitch/yaw relative to startup (degrees) |
| `depth` | std_msgs/Float64 | Depth from barometer |
| `angular_velocity` | geometry_msgs/Twist | Angular rates p/q/r (rad/s) |

| Subscribed Topics | Type | Source |
|---|---|---|
| `joy` | sensor_msgs/Joy | joy_node |
| `cmd_vel` | geometry_msgs/Twist | teleop_twist_joy |
| `imu/data` | sensor_msgs/Imu | MAVROS |
| `global_position/rel_alt` | std_msgs/Float64 | MAVROS |

| Service Clients | Type | Purpose |
|---|---|---|
| `cmd/command` | mavros_msgs/srv/CommandLong | Arm/disarm, servo control |
| `set_stream_rate` | mavros_msgs/srv/StreamRate | Set IMU data rate |

### camera_streamer

Low-latency H.264 video receiver using the GStreamer C API. Publishes via `image_transport` which automatically provides compressed topic variants.

| Published Topics | Type | Description |
|---|---|---|
| `camera/image` | sensor_msgs/Image | BGR8 camera frames |
| `camera/image/compressed` | sensor_msgs/CompressedImage | JPEG compressed (auto) |
| `camera/image/theora` | theora_image_transport/Packet | Theora compressed (auto) |

Features:
- Auto-detects hardware H.264 decoder (VA-API for Intel, NVDEC for NVIDIA), falls back to software
- Watchdog auto-restarts pipeline on stream loss
- Optional frame resize (disabled by default)

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- BlueROV2 with BlueOS

## Installation

### 1. Install ROS 2 dependencies

```bash
sudo apt install \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
  ros-jazzy-cv-bridge
```

### 2. Install GeographicLib datasets

GeographicLib provides geoid, gravity, and magnetic field datasets required by MAVROS for coordinate frame transformations. Even though the BlueROV2 operates underwater without GPS, MAVROS needs these datasets to initialize its global position plugin. Without them, MAVROS will fail to start or produce warnings about missing geoid data.

The datasets include:
- **Geoid models** (egm96-5): Convert between ellipsoidal and orthometric heights
- **Gravity models** (egm96): Earth gravity field calculations
- **Magnetic models** (emm2015): Earth magnetic field calculations

Install them with:

```bash
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

This downloads approximately 200 MB of data to `/usr/share/GeographicLib/`. The script only needs to be run once.

### 3. Install GStreamer development libraries

```bash
sudo apt install \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav
```

For Intel hardware-accelerated H.264 decoding (optional but recommended):

```bash
sudo apt install gstreamer1.0-vaapi
```

### 4. Build the package

```bash
cd ~/your_workspace
colcon build --packages-select bluerov2_controller
source install/setup.bash
```

## Network Setup

Connect to the BlueROV2 via its tether Ethernet interface. Configure the host interface:

| Device | IP Address |
|---|---|
| Host computer | 192.168.2.1 |
| BlueROV2 (BlueOS) | 192.168.2.2 |

Verify connectivity:

```bash
ping 192.168.2.2
```

Access the BlueOS web interface at `http://192.168.2.2` for vehicle configuration, firmware updates, and sensor calibration.

## Usage

### Full system launch

```bash
ros2 launch bluerov2_controller bluerov2_bringup.launch.py
```

### Individual launches

```bash
# MAVROS only
ros2 launch bluerov2_controller mavros.launch.py

# Gamepad only
ros2 launch bluerov2_controller gamepad.launch.py

# Controller only (requires MAVROS running)
ros2 launch bluerov2_controller controller.launch.py

# Camera only
ros2 launch bluerov2_controller camera.launch.py
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `namespace` | `bluerov2` | ROS namespace for all nodes |
| `fcu_url` | `udp://192.168.2.1:14550@192.168.2.2` | MAVROS FCU connection URL |
| `gcs_url` | `udp://@127.0.0.1` | Ground control station URL |
| `run_initialization_test` | `false` | Flash lights and sweep camera on startup |

Example:

```bash
ros2 launch bluerov2_controller bluerov2_bringup.launch.py run_initialization_test:=true
```

## ArduSub Flight Modes

The controller switches ArduSub flight modes on the Pixhawk via the MAVROS `set_mode` service. The flight controller handles all stabilization internally -- RC override inputs are always sent from the joystick, but the FC interprets them differently depending on the active mode.

| Mode | Description |
|---|---|
| **Manual** | Raw RC passthrough. No stabilization. Direct thruster control. |
| **Stabilize** | Auto-levels roll and pitch. Pilot controls yaw, heave, surge, sway. |
| **Depth Hold** | Stabilize + automatic depth holding via pressure sensor. Heave input controls depth change rate. Most commonly used mode for general piloting. |
| **Position Hold** | Holds depth and horizontal position. Requires external positioning (e.g., DVL). |

## Joystick Button Layout (Xbox Controller)

| Input | Action |
|---|---|
| Left Stick | Surge (forward/back) / Sway (lateral) |
| Right Stick | Yaw / Pitch / Roll |
| Left Trigger (LT) | Heave (up/down) via linear.z |
| Start | Arm motors |
| Back | Disarm motors |
| Y | Manual mode (no stabilization) |
| X | Stabilize mode (auto-level roll/pitch) |
| A | Depth Hold mode (stabilize + hold depth) |
| B | Position Hold mode (hold depth + position, needs DVL) |
| LB | Camera tilt up |
| RB | Camera tilt down |
| R3 (right stick click) | Camera tilt reset |
| RT (right trigger) | Increase light |
| LT (left trigger) | Decrease light |

All button mappings are configurable via `config/controller_params.yaml`.

## Configuration

### config/controller_params.yaml

Control loop rate, PWM limits, servo settings, joystick button mappings. See the file for all parameters.

### config/camera_params.yaml

Video port, decoder selection, resize settings, watchdog configuration.

| Parameter | Default | Description |
|---|---|---|
| `port` | `5600` | UDP port for H.264 stream |
| `decoder` | `auto` | `auto`, `vaapih264dec`, `nvh264dec`, or `avdec_h264` |
| `resize_enable` | `false` | Enable frame resizing |
| `resize_width` | `960` | Resize target width |
| `resize_height` | `540` | Resize target height |

### config/xbox_teleop.yaml

Axis mappings and scale factors for `teleop_twist_joy`. Customize for different controllers.

## Camera Streaming

### Decoder selection

With `decoder: "auto"` (default), the node checks for hardware decoders:
1. `vaapih264dec` (Intel VA-API) -- best performance on Intel GPUs
2. `nvh264dec` (NVIDIA NVDEC) -- for NVIDIA GPUs
3. `avdec_h264` (FFmpeg software) -- universal fallback

### Testing without the ROV

Send a test H.264 stream:

```bash
# Terminal 1: Generate test video
gst-launch-1.0 videotestsrc ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=127.0.0.1 port=5600

# Terminal 2: Run camera node
ros2 launch bluerov2_controller camera.launch.py

# Terminal 3: Verify
ros2 topic hz /bluerov2/camera/image
ros2 run rqt_image_view rqt_image_view
```

### Viewing the stream

```bash
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/bluerov2/camera/image
```

## Troubleshooting

### MAVROS won't connect

- Verify network: `ping 192.168.2.2`
- Check BlueOS is running: open `http://192.168.2.2` in a browser
- Verify MAVLink port is not in use by QGroundControl or another GCS
- Check GeographicLib datasets are installed

### No camera frames

- Verify video stream: `gst-launch-1.0 udpsrc port=5600 ! application/x-rtp,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink`
- Check `ros2 topic hz /bluerov2/camera/image` -- the watchdog will auto-restart the pipeline after 6 seconds of no frames
- Try forcing software decoder: set `decoder: "avdec_h264"` in camera_params.yaml

### Joystick not detected

- Check device exists: `ls /dev/input/js*`
- Test with: `jstest /dev/input/js0`
- If permission denied: `sudo chmod 666 /dev/input/js0` or add user to `input` group

### Motors not responding

- Verify the vehicle is armed (press Start button)
- Check mode is MANUAL (press Y button)
- Verify MAVROS connection: `ros2 topic echo /bluerov2/mavros/state`
- Check RC override is publishing: `ros2 topic hz /bluerov2/rc/override`
