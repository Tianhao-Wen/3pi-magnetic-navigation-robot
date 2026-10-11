[3Pi_README.md](https://github.com/user-attachments/files/33295129/3Pi_README.md)
<div align="center">

# 🤖 3Pi+ Autonomous Magnetic Puck Retrieval Robot

**An autonomous mobile robot for magnetic target search, localisation, alignment, and retrieval**

[![Arduino C++](https://img.shields.io/badge/Language-Arduino%20C%2FC%2B%2B-blue?style=flat-square&logo=arduino)](https://www.arduino.cc/)
[![Robot](https://img.shields.io/badge/Robot-Pololu%203Pi%2B-red?style=flat-square)](https://www.pololu.com/)
[![Sensor](https://img.shields.io/badge/Sensor-LIS3MDL%20Magnetometer-orange?style=flat-square)](https://www.pololu.com/product/2737)
[![Control](https://img.shields.io/badge/Control-Odometry%20%2B%20FSM-green?style=flat-square)](#-control-architecture)
[![Result](https://img.shields.io/badge/Best%20Run-11%20Retrievals%20%2F%204%20min-purple?style=flat-square)](#-performance)
[![University](https://img.shields.io/badge/University%20of%20Bristol-Robotics-lightgrey?style=flat-square)](https://www.bristol.ac.uk/)

*Odometry-Guided · Magnetically Sensed · State-Machine Controlled · Fully Autonomous*

</div>

---

## 🎬 Demo

> Add a short demo GIF or video here when publishing the repository.

<!--
Example:

[![Demo Video](./assets/demo_thumbnail.jpg)](./assets/robot_demo.mp4)

or

![3Pi+ Demo](./assets/demo.gif)
-->

**Best recorded run:** **11 successful magnetic-puck retrievals in 4 minutes with no human intervention after launch.**

---

## ✨ Key Features

| Feature | Description |
|---|---|
| 🧭 **Encoder Odometry** | Tracks the robot pose `(x, y, θ)` from left/right wheel encoder increments |
| 🗺️ **Waypoint Search** | Follows a predefined search route while continuously checking for magnetic targets |
| 🧲 **Magnetic Target Detection** | Uses a LIS3MDL 3-axis magnetometer with startup calibration and magnitude-threshold detection |
| 🎯 **Target Position Estimation** | Estimates puck position in front of the robot at the instant of detection |
| 🧮 **Geometry-Based Alignment** | Computes a lateral detour point and a behind-target point before pushing |
| 🔄 **Hierarchical FSM** | Coordinates searching, travelling, alignment, pushing, returning, pausing, and stopping |
| 🎛️ **Closed-Loop Steering** | Corrects wheel PWM from heading error using proportional steering logic |
| 🛑 **Autonomous Safety Logic** | Re-aligns after excessive heading error and stops automatically after the 4-minute run window |

---

## 🏗️ Control Architecture

```text
                     ┌────────────────────────────┐
                     │      Wheel Encoders        │
                     └─────────────┬──────────────┘
                                   │
                                   ▼
                     ┌────────────────────────────┐
                     │ Differential-Drive         │
                     │ Odometry: x, y, theta      │
                     └─────────────┬──────────────┘
                                   │
                 ┌─────────────────┴─────────────────┐
                 │                                   │
                 ▼                                   ▼
      ┌──────────────────────┐            ┌──────────────────────┐
      │ Waypoint Navigation  │            │ LIS3MDL Magnetometer │
      │ heading + distance   │            │ calibrated magnitude │
      └──────────┬───────────┘            └──────────┬───────────┘
                 │                                   │
                 └─────────────────┬─────────────────┘
                                   ▼
                     ┌────────────────────────────┐
                     │ Hierarchical State Machine │
                     │ search / align / push      │
                     └─────────────┬──────────────┘
                                   │
                                   ▼
                     ┌────────────────────────────┐
                     │ PWM Differential Drive     │
                     │ left motor / right motor   │
                     └────────────────────────────┘
```

The control loop updates odometry approximately every **20 ms** and uses the current pose for waypoint navigation, target geometry, alignment, and return-to-start behaviour.

---

## 🔄 Finite-State Machine

The main behaviour is implemented as a seven-state finite-state machine:

```text
SET_WAYPOINT
     │
     ▼
   TRAVEL ────── magnetic target detected ─────► ALIGN
     │                                             │
     │ waypoint reached                            │ BACKOFF
     ▼                                             ▼
SET_WAYPOINT                                  GO_M (side point)
                                                   │
                                                   ▼
                                              GO_E (behind puck)
                                                   │
                                                   ▼
                                             FACE_START
                                                   │
                                                   ▼
                                                 PUSH
                                                   │
                                            puck reaches
                                            start zone
                                                   │
                                                   ▼
                                                 PAUSE
                                                   │
                                                   └────────► SET_WAYPOINT

After all search waypoints are visited:
SET_WAYPOINT ─► RETURN ─► PAUSE ─► SET_WAYPOINT

At 4 minutes:
ANY STATE ─► STOP
```

### Main states

| State | Purpose |
|---|---|
| `SET_WAYPOINT` | Select the next search waypoint or return target |
| `TRAVEL` | Navigate toward the active waypoint while monitoring the magnetometer |
| `ALIGN` | Move around the detected puck and position the robot behind it |
| `PUSH` | Push the puck toward the start zone |
| `RETURN` | Return to the origin after completing the search route |
| `PAUSE` | Stop briefly before beginning another search cycle |
| `STOP` | Stop both motors when the 4-minute run time expires |

### Alignment substates

| Substate | Purpose |
|---|---|
| `BACKOFF` | Reverse away from the puck after detection |
| `GO_M` | Move to a lateral detour point |
| `GO_E` | Move to the point behind the puck |
| `FACE_START` | Rotate to face the start zone before pushing |

---

## 🧲 Magnetometer Calibration & Target Detection

At startup, the robot rotates in place for approximately **4 seconds** while recording the minimum and maximum value on each magnetometer axis.

For each axis:

```text
offset  = min + (max - min) / 2
scaling = 1 / ((max - min) / 2)
```

The calibrated axis value is:

```text
calibrated = (raw - offset) × scaling
```

The calibrated field magnitude is then:

```text
m = sqrt(mx² + my² + mz²)
```

A target is detected when the calibrated magnitude exceeds the tuned threshold:

```cpp
#define DETECT_THRESHOLD 6.69
```

Once triggered, the robot estimates the puck centre approximately **55 mm in front of its current pose**:

```text
puck_x = robot_x + 55 × cos(theta)
puck_y = robot_y + 55 × sin(theta)
```

> The detection threshold is hardware- and environment-dependent and should be re-tuned if the robot, magnet, or operating environment changes.

---

## 🎯 Geometry-Based Target Alignment

Directly pushing a puck from the detection pose can send it away from the start zone.  
Instead, the robot computes two intermediate points:

- **M** — a lateral detour point beside the puck
- **E** — a point behind the puck on the line from the start zone through the puck

```text
                         M
                         ●
                        / \
                       /   \
              robot ●       ● E
                         puck ●
                              \
                               \
                                \
                                 ● Start (0,0)
```

The sequence is:

```text
Detect puck
   ↓
Back off
   ↓
Move to M
   ↓
Move to E
   ↓
Face start zone
   ↓
Push puck toward origin
```

The side used for the detour is selected from the sign of a 2D cross product, allowing the robot to approach from the side that avoids driving through the target.

Key geometry constants in the current implementation:

| Parameter | Value | Meaning |
|---|---:|---|
| `DETECT_DIST` | `55 mm` | Estimated sensor-to-puck distance at detection |
| `SAFE_RADIUS` | `170 mm` | Offset used to construct detour / behind-target points |
| `BACKOFF_DIST` | `70 mm` | Reverse distance before the detour |
| `START_ZONE_RADIUS` | `80 mm` | Radius used to determine a successful return to the start zone |

---

## 🧭 Odometry

The robot estimates its pose from wheel encoder counts.

Current kinematic parameters:

```cpp
const float count_per_rev = 358.3;
const float wheel_radius  = 16.50;  // mm
const float wheel_sep     = 43.5;   // mm from robot centre to wheel centre
```

Encoder counts are converted to travelled distance using:

```text
mm_per_count = 2π × wheel_radius / count_per_rev
```

For each update:

```text
mean_delta = (Δleft + Δright) / 2
Δs         = mean_delta × mm_per_count

Δtheta     = (Δright - Δleft) × mm_per_count
             --------------------------------
                      2 × wheel_sep
```

The pose is then updated in the global frame:

```text
x     ← x + Δs × cos(theta)
y     ← y + Δs × sin(theta)
theta ← theta + Δtheta
```

Odometry is updated approximately every **20 ms**.

> Wheel slip and encoder integration cause accumulated pose error over time, so the controller includes heading re-alignment during waypoint travel.

---

## 🎛️ Navigation & Steering

### Heading control

The robot wraps heading error to `[-π, +π]`:

```cpp
float shortestAngleDifference(float target, float current) {
    float diff = target - current;
    return atan2(sin(diff), cos(diff));
}
```

A proportional steering signal is then applied to the two wheels:

```text
turn_pwm ∝ heading_error

left_pwm  = forward_pwm - turn
right_pwm = forward_pwm + turn
```

### Travel logic

Each waypoint is handled in two stages:

```text
TRAVEL_TURN → TRAVEL_DRIVE → TRAVEL_DONE
```

During driving:

- distance to target is continuously recalculated;
- heading error is converted into differential wheel PWM;
- steering is limited to avoid overly aggressive corrections;
- if heading error exceeds the configured limit, the robot stops forward travel and re-enters the turning state;
- a distance-dependent PWM profile is used near the waypoint to maintain reliable motion.

---

## 🗺️ Search Waypoints

The current search route uses six non-origin waypoints:

| Index | X (mm) | Y (mm) |
|---:|---:|---:|
| 1 | 142 | 258 |
| 2 | 270 | 159 |
| 3 | 396 | 275 |
| 4 | 419 | 70 |
| 5 | 275 | -20 |
| 6 | 145 | 100 |

The robot continuously checks the magnetic field while travelling between these points.  
If no puck is detected, it proceeds to the next waypoint. After completing the route, it returns to the origin and begins another search cycle.

---

## 📁 File Structure

```text
3Pi+robot/
└── 3Pi_CodeStub_wen/
    ├── 🤖 3Pi_CodeStub.ino   # Main autonomous behaviour, FSM, navigation and target retrieval logic
    ├── 🧭 Kinematics.h       # Differential-drive odometry / pose estimation
    ├── 🧲 Magnetometer.h     # LIS3MDL sensor access and calibrated magnitude calculation
    ├── ⚙️ Motors.h           # Differential motor PWM and direction control
    ├── 🔢 Encoders.h         # Wheel encoder interrupt / count support
    └── 🖥️ oled.h             # OLED wrapper and run countdown support
```

---

## ⚙️ Requirements

### Hardware

- Pololu **3Pi+** differential-drive mobile robot
- Left / right wheel encoders
- **LIS3MDL** 3-axis magnetometer
- OLED display
- Buzzer
- Magnetic puck / target objects

### Software

- Arduino IDE
- C/C++ / Arduino toolchain
- `LIS3MDL` library
- `PololuOLED` library

---

## 🚀 Running the Robot

### 1. Clone the repository

```bash
git clone <your-repository-url>
cd <your-repository-name>/3Pi_CodeStub_wen
```

### 2. Open the sketch

Open:

```text
3Pi_CodeStub.ino
```

in the Arduino IDE.

### 3. Install required libraries

Install the libraries used by:

```cpp
#include <LIS3MDL.h>
#include <PololuOLED.h>
```

using the Arduino Library Manager or the appropriate Pololu library package.

### 4. Connect and upload

1. Connect the 3Pi+ to the computer.
2. Select the correct board / processor configuration for the robot.
3. Select the correct serial port.
4. Compile and upload `3Pi_CodeStub.ino`.

### 5. Start the autonomous run

After boot:

```text
Initialise hardware
      ↓
Initialise encoders and odometry
      ↓
4 s magnetometer spin calibration
      ↓
Return to theta = 0
      ↓
Reset pose to (0, 0, 0)
      ↓
Start waypoint search
      ↓
Detect → align → retrieve → repeat
      ↓
Stop automatically at 4 minutes
```

No manual control is required after launch.

---

## 🔧 Important Tuning Parameters

The current values were tuned for the project robot and test environment.

| Parameter | Current value | Purpose |
|---|---:|---|
| `RUN_TIME` | `240000 ms` | Total autonomous run time |
| `POSE_UPDATE` | `20 ms` | Odometry update interval |
| `DETECT_THRESHOLD` | `6.69` | Magnetic target detection threshold |
| `CALIBRATE_TIME` | `4000 ms` | Magnetometer calibration duration |
| `TURN_THRESHOLD` | `0.05 rad` | Heading tolerance |
| `TURN_GAIN` | `118.0` | In-place turning proportional gain |
| `TRAVEL_TOLERANCE` | `21 mm` | Waypoint arrival tolerance |
| `MOVE_GAIN` | `40.0` | Travel steering gain |
| `RETURN_TO_TURN_ANGLE` | `0.5 rad` | Re-align threshold during travel |
| `SAFE_RADIUS` | `170 mm` | Target avoidance / alignment geometry |
| `BACKOFF_DIST` | `70 mm` | Reverse distance after detection |
| `PUSH_PWM` | `20` | Forward PWM while pushing |
| `START_ZONE_RADIUS` | `80 mm` | Puck-return completion radius |
| `PAUSE_TIME` | `4000 ms` | Pause between search cycles |

> These constants should be re-tuned after changing wheel geometry, surface material, battery level, sensor mounting, or puck magnet strength.

---

## 📊 Performance

### Best recorded autonomous run

| Metric | Result |
|---|---:|
| Run duration | **4 minutes** |
| Successful target retrievals | **11** |
| Human intervention after launch | **0** |
| Navigation | Encoder odometry + waypoint guidance |
| Target sensing | Calibrated LIS3MDL magnetometer |
| Behaviour coordination | Hierarchical finite-state machine |

The result was obtained through repeated tuning of:

- magnetic detection threshold;
- wheel PWM and heading gains;
- waypoint locations;
- target approach geometry;
- back-off distance;
- start-zone stopping threshold;
- state transition logic.

---

## 🧪 Debugging & Validation Strategy

During development, the system was decomposed into independently testable behaviours:

1. **Motor direction and PWM**
2. **Encoder counting**
3. **Odometry update**
4. **In-place heading control**
5. **Waypoint travel**
6. **Magnetometer calibration**
7. **Magnetic-threshold detection**
8. **Target geometry calculation**
9. **Alignment substates**
10. **Push-to-start behaviour**
11. **Full 4-minute FSM run**

This modular structure makes it easier to isolate whether a failure is caused by:

- sensing;
- pose estimation;
- navigation;
- geometry;
- actuator control; or
- state-transition logic.

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| 🤖 Robot | Pololu 3Pi+ |
| 💻 Language | Arduino C/C++ |
| 🧭 Localisation | Differential-drive encoder odometry |
| 🧲 Target sensing | LIS3MDL 3-axis magnetometer |
| 🎛️ Control | Proportional heading / steering control + PWM |
| 🧠 Behaviour logic | Hierarchical finite-state machine |
| 📐 Geometry | `atan2`, Euclidean distance, 2D cross product |
| 🖥️ Interface | OLED countdown + buzzer feedback |

---

## 🏷️ Keywords

`autonomous mobile robot` `3Pi+` `Arduino` `embedded robotics` `differential drive` `odometry` `finite state machine` `FSM` `magnetometer` `LIS3MDL` `waypoint navigation` `robot control` `PWM` `sensor calibration` `target retrieval`

---

## 🙏 Acknowledgements & Code Provenance

This project was developed as an individual robotics project at the **University of Bristol**.

The repository contains project-specific autonomous control logic together with course support / starter components. In particular, comments in the supplied source identify support code such as the encoder interrupt implementation and OLED wrapper.

The main project-specific behaviour is implemented in `3Pi_CodeStub.ino`, including:

- startup magnetometer calibration;
- waypoint search behaviour;
- heading control;
- magnetic target detection;
- puck position estimation;
- geometry-based target alignment;
- hierarchical state-machine logic;
- push-to-start behaviour;
- repeated autonomous search cycles; and
- four-minute autonomous stopping logic.

> If publishing this repository publicly, verify that redistribution of any course-provided starter files is permitted by the relevant course / university policy.

---

## 📜 Usage

This repository is intended for **academic portfolio and educational reference**.

Before adding an open-source license, confirm that any included course-provided support code may be redistributed under that license.
