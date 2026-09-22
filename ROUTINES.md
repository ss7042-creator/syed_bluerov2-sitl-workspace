# ROS2 Autonomous Control - Body-Frame Motion via MAVROS

> A complete guide for understanding and implementing **autonomous body-frame motion control** of a BlueROV2 Heavy using ROS2, MAVROS, and ArduSub rc/override - designed to work alongside the SITL + Gazebo setup guide.

<div align="center">

**[Mental Model](#mental-model--how-control-works)** • **[Why rc/override](#why-rcoverride)** • **[Channel Mapping](#part-1--channel-mapping-bluerov2-heavy)** • **[Flight Mode](#part-2--flight-mode-is-everything)** • **[Architecture](#part-3--architecture--the-three-file-design)** • **[MotionPrimitives](#part-4--motionprimitives)** • **[Routines](#part-5--routines)** • **[Running It](#part-6--running-autonomous-control)**

</div>

---

## Mental Model - How Control Works

Before touching any code, understand what the control path actually looks like when you publish from ROS2.

### Control Path: ROS2 → ArduSub → Gazebo

```
        YOUR CODE (Routines / MotionPrimitives)
                       │
             ROS2 Publisher Node
                       │ /mavros/rc/override  ← OverrideRCIn message
                    MAVROS2
                       │ MAVLink RC_CHANNELS_OVERRIDE
                  ArduSub SITL
                       │ thruster mixing (frame-aware)
             ArduPilotPlugin (JSON socket)
                       │
                  Gazebo Sim 8
                       │
        BlueROV2 Heavy Physics (8 thrusters)
```

**Key Insight:**
> `rc/override` sends synthetic joystick values to ArduSub. ArduSub treats them exactly as if a pilot moved the sticks on a physical RC transmitter. The flight controller handles all thruster mixing internally - your code never needs to know which motor spins in which direction.

### Why This Is Different From `cmd_vel`

The SITL setup guide uses MAVROS for **telemetry observation** (reading pose, IMU, state). This guide is about **control** - and control requires a different approach:

| Topic | Frame | Mode Required | Needs GPS/Position |
|-------|-------|---------------|-------------------|
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | World / Odom | GUIDED | ✅ Yes |
| `/mavros/rc/override` | Body (robot-relative) | MANUAL / STABILIZE / ALT_HOLD | ❌ No |

When you publish `forward = 1.0` on `cmd_vel`, the robot moves in the **world +X direction** regardless of where it is pointing. When you publish the same intent via `rc/override` channel 5, the robot moves **in the direction it is currently facing**. For an ROV without GPS, `rc/override` is the correct and standard approach.

---

## Why rc/override

There are three ways to send motion commands through MAVROS. Understanding why we use `rc/override` and not the others is important for debugging and future development.

### Option 1 - `setpoint_velocity/cmd_vel` (world frame)

- Requires **GUIDED mode** and an active position estimate (GPS, DVL, or USBL)
- Commands are in the **world/odom frame**: "move at 0.5 m/s along world X"
- After a rotation, "forward" still means world X - the robot crabs sideways
- ❌ Not suitable for GPS-less SITL or real BlueROV2 without localization

### Option 2 - `setpoint_raw/attitude` (attitude setpoints)

- Controls orientation directly via quaternion + collective thrust
- Better suited to PX4 than ArduSub; limited support in ArduPilot's GUIDED implementation
- ❌ Not the standard for ArduSub ROV control

### Option 3 - `rc/override` (body frame) ✅

- Sends synthetic PWM joystick values to ArduSub (1100-1900, neutral 1500)
- Works in **MANUAL, STABILIZE, ALT_HOLD** - no position system needed
- ArduSub interprets channel 5 as "robot forward" and channel 6 as "robot lateral" always
- The flight controller does all frame-aware thruster mixing
- ✅ Correct BKM for GPS-less body-frame ROV control

---

## Part 1 - Channel Mapping (BlueROV2 Heavy)

With `rc/override` chosen as the control method, the next thing to understand is what the channels actually mean for this specific vehicle.

The BlueROV2 Heavy uses 8 thrusters in a vectored configuration giving full 6DOF control. ArduSub maps RC channels to vehicle axes as follows.

### Why Heavy Is Different From Standard

The standard BlueROV2 (6 thrusters, `vectored` frame) uses pitch and roll channels (ch1/ch2) to produce forward and lateral motion via vectored thrust. The Heavy (`vectored_6dof` frame) has **dedicated horizontal thrusters** on ch5 and ch6, making the mapping clean and explicit.

| Channel | Axis | Neutral PWM | Used for Locomotion |
|---------|------|-------------|-------------------|
| ch1 | Pitch | 1500 | ❌ Trim only - hold at neutral |
| ch2 | Roll | 1500 | ❌ Trim only - hold at neutral |
| ch3 | Throttle (Z) | 1500 | ✅ Vertical (up/down) |
| ch4 | Yaw | 1500 | ✅ Rotation |
| ch5 | Forward (body X) | 1500 | ✅ Forward/backward |
| ch6 | Lateral (body Y) | 1500 | ✅ Left/right strafe |

**Key Insight:**
> On the Heavy frame, ch1 and ch2 are deliberately held at 1500 (neutral). Sending non-neutral pitch/roll values does not produce locomotion - it fights the attitude controller. Only channels 3, 4, 5, and 6 matter for motion.

### PWM Conversion

ArduSub expects PWM values between 1100 and 1900. The conversion from a normalized -1.0 to 1.0 range is:

```
PWM = 1500 + (value × 400)

value = -1.0  →  PWM = 1100  (full reverse)
value =  0.0  →  PWM = 1500  (neutral / stop)
value = +1.0  →  PWM = 1900  (full forward)
```

---

## Part 2 - Flight Mode Is Everything

Knowing the channel mapping is not enough — ArduSub will silently ignore `rc/override` if the vehicle is in the wrong flight mode. This is the most common failure point: the topic shows 1 publisher, 1 subscriber, values changing at 10 Hz, and the robot does not move.

### The Rule

| Flight Mode | rc/override Works | Notes |
|-------------|-------------------|-------|
| `GUIDED` | ❌ No | Ignores RC override; expects setpoint topics with a position source |
| `MANUAL` | ✅ Yes | Raw pass-through, no stabilization |
| `STABILIZE` | ✅ Yes | Auto-levels roll/pitch; recommended for most tasks |
| `ALT_HOLD` | ✅ Yes | Stabilize + holds depth; best when only horizontal motion is needed |

### Switch Mode from MAVProxy Terminal

```bash
mode manual
```

### Verify the Mode Actually Changed

```bash
ros2 topic echo /mavros/state --once
```

Look for the `mode` field — it must not say `GUIDED`:

```yaml
connected: true
armed: true
guided: false
mode: MANUAL
```

---

## Part 3 - Architecture - The Three-File Design

With the vehicle layer understood (what topic, what channels, what mode), the question becomes how to structure the code that drives it. The goal is clean separation of concerns: each layer knows only what it needs to, and `MotionPrimitives` acts as a stable, reusable interface that can be called from simple routines today and from full autonomy stacks, path planners, perception pipelines, state machines tomorrow. Adding a new behavior never requires touching the vehicle layer, and the vehicle layer never needs to know what called it.

### Separation of Concerns

Each file has one job:

- **`routine_node.py`** handles ROS2 infrastructure —> services, executors, lifecycle
- **`routines.py`** handles spatial logic —> what sequence of moves constitutes a behavior
- **`motion_primitives.py`** handles vehicle translation —> turning directional intent into PWM

This means a path planner, a perception-triggered reaction, or a state machine can all call the same `MotionPrimitives` interface directly, without touching or duplicating any vehicle-level code. The interface is stable; the algorithms above it can evolve freely.

### Interaction Execution Flow

```
[ User / Algorithm ] ──→ Trigger Routine (e.g., "square")
                                     │
                                     ▼
                      +──────────────────────────────+
                      |       routine_node.py        |
                      |   (MultiThreadedExecutor)    |
                      +──────────────────────────────+
                                     │
                           Invokes routine method
                                     ▼
                      +──────────────────────────────+
                      |         routines.py          |
                      |  (Sequence: Fwd → Rotate)    |
                      +──────────────────────────────+
                                     │
                         Updates shared command state
                                     ▼
                      +──────────────────────────────+
                      |     motion_primitives.py     |
                      |  (Main thread blocks/sleeps) |
                      +──────────────────────────────+
                                     │
                          Safe mutex lock handshake
                                     ▼
                      +──────────────────────────────+
                      |    BACKGROUND DAEMON THREAD  |
                      |   (Strict 10 Hz MAVROS loop) |
                      +──────────────────────────────+
                                     │
                          Publishes OverrideRCIn msg
                                     ▼
                               [ MAVROS Node ]
```

### What Each File Does

**`routine_node.py` — The ROS2 Infrastructure**

Registers the ROS2 service and assigns incoming navigation workflows to a `MultiThreadedExecutor`. This allocates a pool of threads so that long-duration trajectory scripts never freeze incoming status checks or background loops. The service callback runs in its own thread, the rest of the ROS2 ecosystem keeps spinning. In a future autonomy stack, this is where a mission manager, action server, or behavior tree would plug in.

**`routines.py` — The Behavior Layer**

Coordinates abstract spatial geometry paths (square, circle, dive, scan). Calls lower-level commands sequentially. Entirely decoupled from raw PWM mappings, threading, and network constraints. Adding a new behavior, a spiral search, a docking approach means adding a method here and nothing else. This layer could equally be driven by a planner or a perception pipeline calling the same primitives.

**`motion_primitives.py` — The Vehicle Interface**

The stable, reusable boundary between any algorithm and the physical vehicle. Translates directional intent (`forward`, `rotate_left`, etc.) into raw PWM signals and owns all MAVLink publish mechanics. Internally it runs a background daemon thread at a strict 10 Hz cadence so that blocking calls in the layer above never starve the MAVLink stream, a practical consequence of keeping the interface clean and simple. A mutex ensures safe handoff between the calling thread and the daemon. Nothing above this layer ever touches PWM values or publish rates.

| File | Knows About PWM | Knows About ROS2 | Blocks/Sleeps | Publishes |
|------|----------------|-----------------|---------------|-----------|
| `routine_node.py` | ❌ | ✅ | ❌ | ❌ |
| `routines.py` | ❌ | ❌ | ✅ | ❌ |
| `motion_primitives.py` | ✅ | ✅ | Main thread only | ✅ daemon |

---

## Part 4 - MotionPrimitives

`MotionPrimitives` is the complete API for vehicle motion. Every primitive follows the same signature — `(duration: float, speed: float)` where speed is normalized 0.0-1.0, so any algorithm calling this layer has a uniform, predictable interface.

### Available Primitives

| Method | Axis | Channel | Direction |
|--------|------|---------|-----------|
| `forward(duration, speed)` | Body X | ch5 | + forward |
| `backward(duration, speed)` | Body X | ch5 | - forward |
| `left(duration, speed)` | Body Y | ch6 | + lateral |
| `right(duration, speed)` | Body Y | ch6 | - lateral |
| `ascend(duration, speed)` | Z | ch3 | + vertical |
| `descend(duration, speed)` | Z | ch3 | - vertical |
| `rotate_left(duration, speed)` | Yaw | ch4 | + yaw |
| `rotate_right(duration, speed)` | Yaw | ch4 | - yaw |

### Internal Design

| Decision | Reason |
|----------|--------|
| Background daemon at 10 Hz | Lets callers use plain blocking calls without worrying about MAVLink keepalive, the interface stays simple for any algorithm above it |
| `threading.Lock()` mutex | Ensures the daemon never reads a half-written command during a step transition |
| `stop()` resets to neutral | Safe boundary between every primitive call; daemon applies it within 100 ms |
| All 18 channels reset on stop | Resets every possible channel, not just the 6 used |
| `max_vertical = 0.7` | Vertical thrusters are weaker on Heavy; 0.7 avoids saturation |
| Speed clamped to -1.0..1.0 | Any caller passing out-of-range values never produces invalid PWM |

---

## Part 5 - Routines

Routines are high-level sequences of primitives. They have no knowledge of PWM, channels, threads, or MAVROS, they only call the `MotionPrimitives` interface. The dispatcher pattern means any routine can be triggered by name from a service call, a planner, or a CLI.

### Available Routines

| Routine | Character | Depth Changes | Speed | Use Case |
|---------|-----------|---------------|-------|----------|
| `square` | Deterministic | ❌ | Medium | Position verification, repeatability tests |
| `circle` | Deterministic | ❌ | Medium | Arc tracking, area coverage |
| `random` | Aggressive random | ✅ | Medium-fast | Unknown space exploration |
| `random_drift` | Biased forward | Rare | Slow | Survey, inspection passes |
| `dive` | Deterministic | ✅ | Slow | Depth transition testing |
| `spin` | Deterministic | ❌ | Slow | 360° scan, situational awareness |

### Adding a New Routine

Add a method to `routines.py` and register it in the `registry` dict. No other file changes. The new routine is immediately available via the service interface.

```python
def my_routine(self):
    self.c.forward(2.0, 0.5)
    self.c.rotate_left(1.4, 0.5)
    self.c.descend(1.5, 0.4)

# register:
self.registry["my_routine"] = self.my_routine
```

---

## Part 6 - Running Autonomous Control

This section assumes the SITL + Gazebo stack from the setup guide is already running. Pick up from the point where the vehicle is armed.

### Prerequisites

```
✅ Terminal 1: Gazebo running with bluerov2_heavy_underwater.world
✅ Terminal 2: ArduSub SITL running with -f vectored_6dof --model=JSON
✅ MAVProxy: output add udp:127.0.0.1:14551 done
✅ Terminal 3: MAVROS2 node running on port 14551
✅ Vehicle: armed
```

### Step 1 - Switch to MANUAL Mode

In the MAVProxy console:

```bash
mode manual
```

### Step 2 - Verify rc/override Topic Exists

```bash
ros2 topic list | grep rc
# /mavros/rc/override
# /mavros/rc/in
# /mavros/rc/out
```

### Step 3 - Run Your Node

```bash
ros2 run bluerov2_routines routine_node
```

### Step 4 - Call Routine Service

```bash
ros2 service call /execute_routine bluerov2_interfaces/srv/ExecuteRoutine "{routine_name: square}"
```

### Step 5 - Confirm Motion in Gazebo

Watch the vehicle move in the Gazebo viewport. Verify the rc/override values are changing:

```bash
ros2 topic echo /mavros/rc/override
# channels: [1500, 1500, 1500, 1500, 1900, 1500, ...]
#                                           ^^^^ ch5 = full forward
```

Confirm rc/out is responding (thruster outputs are changing):

```bash
ros2 topic echo /mavros/rc/out
```

---

## Key Takeaways

| # | Principle | Impact |
|---|-----------|--------|
| 1 | **MotionPrimitives is the stable vehicle interface** | Any algorithm, routines, planners, perception reactions calls the same 8 methods without knowing anything about PWM or MAVLink |
| 2 | **Each layer has one job** | Behaviors live in `routines.py`, infrastructure in `routine_node.py`, vehicle translation in `motion_primitives.py` — adding a behavior never touches the vehicle layer |
| 3 | **rc/override is body-frame by design** | Forward always means robot forward, regardless of heading |
| 4 | **Flight mode is the gatekeeper** | GUIDED ignores rc/override entirely, MANUAL or STABILIZE required |
| 5 | **ch1/ch2 are unused on Heavy** | Pitch and roll channels are for attitude trim, not locomotion |
| 6 | **The daemon thread is an implementation detail** | It exists so callers use plain blocking calls without worrying about MAVLink keepalive, the interface stays clean |
| 7 | **Routines are just primitives in sequence** | No MAVROS, PWM, or thread knowledge required above MotionPrimitives |