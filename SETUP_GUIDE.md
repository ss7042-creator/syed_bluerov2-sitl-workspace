# ArduPilot SITL + Gazebo Harmonic - Setup & Field Guide

> A complete guide for setting up and running **BlueROV2 Heavy underwater vehicle** simulations with ArduPilot, Gazebo, ROS2, and joystick control.

<div align="center">

**[Mental Model](#mental-model--how-the-stack-works)** • **[Installation](#installation)** • **[Running Simulation](#part-5--running-the-underwater-simulation-bluerov2-heavy)** • **[MAVROS Integration](#part-6--mavros2-integration)** • **[Joystick Control](#part-7--xbox-controller-integration)**

</div>

---

## Mental Model - How the Stack Works

Before touching the terminal, understand what each piece does and why it exists.

### System Architecture: ArduSub + BlueROV2 Heavy

```
        YOU (pilot commands)
               │
 MAVProxy CLI / QGroundControl    ← Control interface
               │ MAVLink
         ArduSub SITL            ← Vehicle brain
               │ JSON socket
       ArduPilotPlugin           ← Bridge
    (inside BlueROV2 model SDF)
               │
         Gazebo Sim 8            ← Physics engine
               │
 bluerov2_heavy_underwater.world
     • BuoyancyPlugin            ← Neutral buoyancy
     • HydrodynamicsPlugin       ← Water resistance
     • ThrusterPlugin            ← 8-thruster vectored control
```

**Key Insight:**
> None of these layers work in isolation. Gazebo provides physics but lacks vehicle logic. SITL provides vehicle logic but lacks physics. `ArduPilotPlugin` is the glue. `bluerov2_gz` adds all underwater-specific physics that vanilla Gazebo doesn't include.

---

# Installation

## Part 1 - Install ArduPilot

You can follow the official instructions on the [official website](https://ardupilot.org/dev/docs/building-setup-linux.html), or follow the steps below:

#### Clone the Repository

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
cd ardupilot
```

> `--recurse-submodules` is required. ArduPilot uses git submodules for several dependencies. Skipping this flag means those components won't be present, and the build will fail.

#### Install Dependencies

Run the provided script from inside the cloned directory:

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Then reload your PATH:

```bash
. ~/.profile
```

> **Why reload?** The script adds new tools to your PATH. The change isn't active until the profile is re-sourced. Log out and back in to make it permanent.

#### Build SITL

```bash
./waf configure --board sitl
./waf sub       # for ArduSub (underwater)
```

> `--board sitl` tells the build system to compile for software-in-the-loop simulation instead of real hardware. No cross-compilation, just native x86 binaries that run on your machine and pretend to be a flight controller.

---

## Part 2 - Install Gazebo Harmonic + ArduPilot Plugin

#### Install Gazebo Dependencies

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-libav gstreamer1.0-gl
```

Verify your Gazebo version:

```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

#### Clone and Build the ArduPilot Gazebo Plugin

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

> This compiles `libArduPilotPlugin.so` - the shared library that Gazebo loads at runtime when it finds the plugin tag inside a vehicle model SDF. Without this file, Gazebo has no idea how to talk to ArduPilot, for any vehicle.

#### Set Environment Variables

Add these to `~/.bashrc`, then source it. This block covers both the iris and BlueROV2 repos:

```bash
# ArduPilot plugin binary
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc

# Models and worlds - BlueROV2 (bluerov2_gz)
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$HOME/bluerov2_gz/models:$HOME/bluerov2_gz/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

source ~/.bashrc
```

> **Why these matter:**
> - `GZ_SIM_SYSTEM_PLUGIN_PATH` - tells Gazebo where to find `libArduPilotPlugin.so`. If unset, the plugin silently fails to load and nothing connects.
> - `GZ_SIM_RESOURCE_PATH` - tells Gazebo where to find world and model SDF files. Both repos need to be on this path.
>
> Do **not** try to source `/usr/share/gz/setup.sh` - that file does not exist in Harmonic.

---

## Part 3 - Install BlueROV2 Gazebo Package

#### Why This Repository Exists

After setting up `ardupilot_gazebo`, you have a working bridge between Gazebo and ArduPilot, but you still have no underwater vehicle. The `bluerov2_gz` repository is what actually adds:

- BlueROV2 and BlueROV2 Heavy model geometry
- Underwater world environments
- Buoyancy, hydrodynamics, and drag physics
- Thruster layout and plugin configuration
- ArduSub integration examples

Without it, there is nothing to simulate underwater.

#### Clone the Repository

```bash
cd ~
git clone https://github.com/clydemcqueen/bluerov2_gz.git
```

Key folders inside:

```
models/    ← BlueROV2 SDF models (standard, Heavy, Ping sonar variants)
worlds/    ← underwater world files
scripts/   ← helper launch scripts
```

> The environment variables you set in Part 2 already include `$HOME/bluerov2_gz/models` and `$HOME/bluerov2_gz/worlds`, so Gazebo will find these automatically after sourcing `~/.bashrc`.

#### Frame Variants

| Frame flag | Vehicle |
|-----------|---------|
| `vectored` | BlueROV2 standard (6 thrusters) |
| `vectored_6dof` | BlueROV2 **Heavy** (8 thrusters, full 6DOF control) |

---

## Part 4 - Install QGroundControl

QGroundControl (QGC) is a GUI ground station that connects over MAVLink-the same protocol MAVProxy uses. You can use either or both: QGC provides a map, telemetry gauges, and joystick support; MAVProxy provides a CLI and scripting capabilities.

### 4.1 Download and Install

```bash
# Download the AppImage from the official site
# https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

> On Ubuntu, you may need: `sudo apt install libfuse2` for AppImage support.

### 4.2 Connect QGC to SITL

When `sim_vehicle.py` starts (SITL code), it automatically opens a MAVLink output on UDP port `14550`. QGC listens on that port by default-so if QGC is running when you launch SITL, it connects automatically with no extra configuration.

```
SITL  →  UDP :14550  →  QGroundControl
SITL  →  UDP :14551  →  MAVProxy (also listening)
```

Both can be connected at the same time. Commands typed in MAVProxy are reflected in QGC and vice versa.

---

## Part 5 - Running the Underwater Simulation (BlueROV2 Heavy)

### 5.1 Terminal 1 - Launch Gazebo with Underwater World

```bash
gz sim -v3 -r bluerov2_heavy_underwater.world
```

| Flag | Meaning | Purpose |
|------|---------|----------|
| `-v4` | Verbosity level 4 | Shows plugin loading, IMU topics, connection events |
| `-r` | Run immediately | Don't pause waiting for Play |

**Warning** until the GUI appears and IMU topics advertise before starting SITL.

#### Available World Environments

| World File | Configuration | Use Case |
|-----------|-------------|----------|
| `bluerov2_underwater.world` | Standard | Standard BlueROV2 (6 thrusters) |
| `bluerov2_heavy_underwater.world` | Heavy | 8-thruster configuration (full 6DOF) |
| `bluerov2_ping.world` | With Sensor | Ping sonar sensor integration |

### 5.2 Terminal 2 - Launch ArduSub SITL

#### SITL Launch Flags

| Flag | Meaning | Configuration | Notes |
|------|---------|---------------|-------|
| `-L RATBeach` | Location | Coastal test site | Suitable for submarine testing |
| `-v ArduSub` | Firmware | Submarine controller | Depth hold, stabilization, buoyancy |
| `-f vectored_6dof` | Frame | Heavy configuration | 8-thruster full 6DOF control |
| `--model=JSON` | Protocol | MAVLink dialect | Required for Gazebo plugin communication |

> **Tip:** Use `-f vectored` for standard BlueROV2 (6 thrusters). All three flags must match your vehicle configuration.

#### MAVProxy Control Commands

```bash
mode manual         # Direct thruster control
mode stabilize      # Attitude hold with auto-leveling
mode alt_hold       # Depth hold (altitude for aircraft)
arm throttle        # Arm vehicle for control
```

#### QGroundControl Console

**Automatic Connection:** QGC connects automatically on UDP `14550`. For BlueROV2 operations, QGC provides:
- Real-time depth, heading, battery status display
- Joystick/gamepad control
- Visual telemetry and map interface

**Recommended for:** Hands-on piloting and real-time monitoring

---

## Quick-Start Card - Basic Setup (BlueROV2 Heavy)

<details open>
<summary><b>Minimal Setup: Gazebo + SITL</b></summary>

```bash
# Terminal 1: Launch Gazebo Physics Engine
gz sim -v3 -r bluerov2_heavy_underwater.world

# Terminal 2: Launch ArduSub SITL Controller
cd ~/ardupilot
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON --console

# In MAVProxy Console: Prepare to Dive
mode stabilize
arm throttle

# Optional: Use QGroundControl + Joystick for Full Control
```

</details>

---

## Part 6 - MAVROS2 Integration (ROS2 ↔ MAVLink Bridge)

**Overview:** MAVROS2 is a ROS2 node that speaks MAVLink. It sits alongside QGroundControl and MAVProxy-all three can be connected simultaneously, each on its own UDP port. 

**Use Case:** MAVROS provides programmatic access, telemetry introspection, and autonomous behavior (not for manual control).

### 6.1 System Architecture: MAVROS with QGC + MAVProxy

```
        YOU (pilot commands)
               │
    ┌───────┐───────┐───────┐
    │       │         │
   QGC    MAVProxy   MAVROS2
  :14550   :14550     :14551  ← UDP ports
    │       │         │
    └───────┈───────┈───────┘
               │ MAVLink protocol
         ArduSub SITL  ← Vehicle brain
               │ JSON socket
         ArduPilotPlugin
               │
           Gazebo Sim 8  ← Physics engine
```

**Key Points:**
- **QGC** = Human control interface (pilots the vehicle)
- **MAVROS** = ROS2 programmatic interface (reads telemetry)
- **Each on separate port** = No interference between clients

### 6.2 Step 1 - Configure MAVProxy Output Ports

**Objective:** Add a dedicated output port for MAVROS (in addition to QGC)

Inside the MAVProxy console:

```bash
output add udp:127.0.0.1:14551
```

#### UDP Port Allocation

| Client | Port | Purpose |
|--------|------|----------|
| **QGroundControl** | `14550` | GUI control interface |
| **MAVROS2** | `14551` | ROS2 bridge (telemetry) |

> **Note:** Both clients can connect simultaneously. Commands via QGC propagate through MAVProxy to SITL, while MAVROS observes the same MAVLink stream.

### 6.3 Step 2 - Launch MAVROS2 Node

```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14551@ \
  -p tgt_system:=1
```

#### MAVROS Configuration Parameters

| Parameter | Value | Meaning |
|-----------|-------|----------|
| `fcu_url` | `udp://:14551@` | Listen for MAVLink on UDP port 14551 |
| `tgt_system` | `1` | Target ArduSub system ID (always 1 in SITL) |

### 6.4 Step 3 - Verify MAVLink Connection

**Expected Output:**
```
CON: Got HEARTBEAT
```

This indicates MAVROS is successfully receiving the MAVLink stream from MAVProxy.

### 6.5 Step 4 - Validate ROS2 Topics

**List all MAVROS topics:**
```bash
ros2 topic list | grep mavros
```

#### Expected MAVROS Topics

| Topic | Data Type | Purpose |
|-------|-----------|----------|
| `/mavros/state` | State | Connection and mode status |
| `/mavros/local_position/pose` | Pose | Real-time 3D position and orientation |
| `/mavros/imu/data` | IMU | Accelerometer, gyroscope, magnetometer |

**Query connection state:**
```bash
ros2 topic echo /mavros/state
```

**Expected output:**
```yaml
connected: true
armed: false
mode: GUIDED
```

### 6.6 Step 5 - Verify Real-Time Synchronization

**Test:** With QGC open, change the vehicle mode or send a command. Then monitor:

```bash
ros2 topic echo /mavros/local_position/pose
```

**Expected behavior:**
- Pose values update in real-time
- Reflects vehicle state changes from QGC commands
- Confirms full data path: QGC → MAVProxy → SITL → Gazebo → ArduPilotPlugin → MAVROS → ROS2

---

## Part 7 - Xbox Controller Integration (Physical Joystick via QGC)

The Xbox controller control path goes entirely through QGroundControl. MAVROS is not involved - it only observes. The controller gives you real-time physical control of the BlueROV2 in Gazebo.

### 7.1 Architecture of This Control Path

```
Xbox Controller
      │  USB / Bluetooth
QGroundControl (axis mapping + calibration)
      │  MAVLink  :14550
MAVProxy
      │
ArduSub SITL
      │
Gazebo physics
```

### 7.2 Step 1 - Verify Linux Detects the Controller

Plug in the Xbox controller (USB or Bluetooth), then:

```bash
ls /dev/input/
```

You should see `js0` in the output. If it's absent, the controller is not recognized by the kernel.

### 7.3 Step 2 - Enable Joystick in QGroundControl

In QGC: **Settings → General → Joystick**

- Enable **Enable Joystick**
- Optionally enable gamepad control mode

> Restart QGC once if the controller was plugged in after QGC was already open.

### 7.4 Step 3 - Calibrate the Joystick (Critical)

**Menu Path:** Settings → Joystick → Calibration

**Calibration Steps:**
1. Move both sticks through full X and Y range
2. Rotate right stick fully for yaw axis
3. Move throttle/depth axis (right stick Y or triggers)
4. Center all sticks
5. Save calibration

**This must be completed before control inputs are registered.**

#### Default Axis Mapping (BlueROV2 Submarine)

| Xbox Control | Vehicle Axis | Action |
|-------------|-------------|--------|
| **Left Stick Y** | Forward/Backward | Move along X-axis |
| **Left Stick X** | Strafe | Lateral movement (Y-axis) |
| **Right Stick Y** | Depth | Ascend/descend (Z-axis) |
| **Right Stick X** | Yaw | Rotate horizontally |

### 7.5 Step 4 - Arm Vehicle and Begin Piloting

**Prerequisites for joystick input to be accepted:**
```bash
In QGC:
  Set mode → MANUAL or STABILIZE
  Arm vehicle
```

> **Important:** Without arming, joystick inputs are ignored regardless of calibration.

### 7.6 Real-Time Feedback Verification

**Observable behavior once active:**

| Component | Expected Behavior |
|-----------|-------------------|
| **Gazebo Simulator** | Vehicle responds physically to stick inputs in real-time |
| **QGC Display** | Live map position, attitude angles, depth gauge all update |
| **MAVROS Topics** | `/mavros/local_position/pose` streams continuous 3D position |

**Verification command:**
```bash
ros2 topic echo /mavros/local_position/pose
```

3D position values change as you move the sticks, confirming full integration.

---

## Complete Setup - Full Stack Quick-Start (BlueROV2 Heavy + MAVROS + Controller)

<details>
<summary><b> Full Integration: All Components</b></summary>

```bash
# TERMINAL 1: Physics Engine
gz sim -v3 -r bluerov2_heavy_underwater.world

# TERMINAL 2: Vehicle Controller
cd ~/ardupilot
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON --console

# IN MAVPROXY: Add ROS2 Bridge Output
output add udp:127.0.0.1:14551

# TERMINAL 3: ROS2 Bridge
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14551@ \
  -p tgt_system:=1

# OPEN: QGroundControl
# - Connects automatically on :14550
# - Plug in Xbox controller
# - Settings → Joystick → Enable + Calibrate
# - Set Mode → MANUAL/STABILIZE → ARM → FLY

# VERIFY ROS2 Bridge
ros2 topic echo /mavros/state                # connected: true
ros2 topic echo /mavros/local_position/pose  # real-time pose
```

</details>

---

## Key Takeaways

| # | Principle | Impact |
|---|-----------|--------|
| 1 | **Plugin lives in model SDF, not world** | `ArduPilotPlugin` is declared inside vehicle model, not world file |
| 2 | **SITL is not a simulator** | Pure control logic; requires Gazebo for physics and sensor data |
| 3 | **Startup order matters** | Gazebo → SITL → MAVROS; reversed order causes connection timeouts |
| 4 | **All SITL flags must match** | `-v`, `-f`, `--model` form a protocol contract; single mismatch = no connection |
| 5 | **Each client needs its own port** | QGC:14550, MAVROS:14551; add ports explicitly via `output add` |
| 6 | **MAVROS observes; QGC controls** | MAVROS is read-only telemetry; Xbox → QGC → MAVLink is active control path |
| 7 | **Controller path bypasses ROS** | Xbox → QGC → MAVProxy → SITL → Gazebo; MAVROS observes in parallel |
