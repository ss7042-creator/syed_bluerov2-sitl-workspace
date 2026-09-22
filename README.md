# bluerov2-sitl-workspace

A ROS2 workspace for simulating and controlling a BlueROV2 Heavy underwater vehicle using ArduSub SITL and Gazebo Harmonic. The goal is to provide a reproducible environment to develop and test autonomous control code without needing physical hardware.

---

## What's in Here

**Simulation environment documentation.** Getting ArduSub SITL, Gazebo Harmonic, the BlueROV2 Heavy model, and MAVROS2 all talking to each other involves several moving parts that are easy to get wrong. The setup guide walks through each dependency in order and explains what each layer does, so the environment is reproducible and failures are diagnosable.

**A basic autonomous control package.** `bluerov2_routines` sends body-frame motion commands to the vehicle via `rc/override` — the appropriate control path for GPS-less ROV operation in SITL. It's structured in three layers so the vehicle interface stays stable and separate from whatever behavior logic runs on top of it. A background 10 Hz publish loop keeps the MAVLink RC stream alive while routines are executing, which avoids triggering ArduSub's RC loss failsafe.

This is an early-stage workspace, primarily intended to help anyone get a working BlueROV2 simulation environment up and running with ROS2 in a way that's actually understood rather than just copy-pasted.

---

## Repository Structure

```
bluerov2-sitl-workspace/
├── automation/
│   ├── install_dependencies.sh        # FUTURE: automated dependency setup
│   └── bringup_simulation.sh          # FUTURE: single-command stack launch
├── bluerov2_interfaces/               # Custom ROS2 service definitions
│   ├── srv/
│   │   └── ExecuteRoutine.srv         # Request: routine_name | Response: success, message
│   ├── CMakeLists.txt
│   └── package.xml
├── bluerov2_routines/                 # Autonomous control package
│   ├── setup.py
│   ├── package.xml
│   └── bluerov2_routines/
│       ├── __init__.py
│       ├── routine_node.py            # ROS2 service server (MultiThreadedExecutor)
│       ├── routines.py                # High-level behavior sequencer
│       └── motion_primitives.py       # Thread-safe 10 Hz vehicle interface
└── README.md
```

---

## How the Stack Fits Together

```
        bluerov2_routines (this repo)
                 │
         MotionPrimitives API
                 │ /mavros/rc/override
              MAVROS2
                 │ MAVLink
           ArduSub SITL              ← vehicle brain
                 │ JSON socket
          ArduPilotPlugin            ← Gazebo bridge
                 │
           Gazebo Sim 8              ← physics engine
                 │
   bluerov2_heavy_underwater.world
      BuoyancyPlugin • HydrodynamicsPlugin • ThrusterPlugin
```

The simulation stack (everything below MAVROS2) is environment infrastructure — set it up once and leave it running. The control package sits on top and talks to the vehicle through MAVROS. The two are decoupled: any node publishing to `/mavros/rc/override` in `MANUAL` or `STABILIZE` mode will move the vehicle.

---

## Where to Start

### 1 — Set Up the Simulation Environment

Start here. The setup guide covers every dependency from scratch: ArduPilot, Gazebo Harmonic, the ArduPilot Gazebo plugin, the BlueROV2 model package, QGroundControl, and MAVROS2. It also covers joystick control via QGC as a sanity check before any autonomous code runs.

> 📄 **[SETUP_GUIDE.md](./SETUP_GUIDE.md)**

### 2 — Run Autonomous Control

Once the simulation is running and the vehicle is armed, the routines guide explains the control interface, the package architecture, and how to trigger and extend routines.

> 📄 **[ROUTINES.md](./ROUTINES.md)**

---

## Quick-Start (Environment Already Set Up)

```bash
# Terminal 1 — physics
gz sim -v3 -r bluerov2_heavy_underwater.world

# Terminal 2 — vehicle brain
cd ~/ardupilot
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON --console

# MAVProxy console
output add udp:127.0.0.1:14551
mode manual
arm throttle

# Terminal 3 — ROS2 bridge
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14551@ \
  -p tgt_system:=1

# Terminal 4 — control node
ros2 run bluerov2_routines routine_node

# Terminal 5 — trigger a routine
ros2 service call /execute_routine bluerov2_interfaces/srv/ExecuteRoutine \
  "{routine_name: square}"
```
