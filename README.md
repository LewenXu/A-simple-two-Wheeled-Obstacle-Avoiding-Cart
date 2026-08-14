
# USYD ELEC1601 Project 

## Overview

This project implements an autonomous maze-navigation controller for a two-wheel servo-driven robot using three infrared (IR) sensors.

The robot continuously senses the environment on its **left**, **front**, and **right** sides, then determines whether to:

- drive forward;
- correct its position inside a corridor;
- turn left;
- turn right;
- identify a T-junction;
- detect and escape from a short dead end;
- stop when a long dead end is interpreted as the finish area.

The navigation logic combines:

- IR wall detection;
- corridor centering;
- confirmed side-opening detection;
- junction handling;
- timed 90° turns;
- U-turn recovery;
- simple navigation memory.

---

## Hardware Configuration

The program uses three IR emitter/receiver pairs, three debugging LEDs, and two continuous-rotation servos.

### IR Sensors

| Direction | IR LED Pin | IR Receiver Pin |
|---|---:|---:|
| Left | 10 | 11 |
| Front | 6 | 7 |
| Right | 2 | 3 |

### Debug LEDs

| Direction | Pin |
|---|---:|
| Left | A2 |
| Front | A1 |
| Right | A0 |

The LEDs indicate whether the corresponding IR sensor currently detects a wall.

### Servo Motors

| Motor | Pin |
|---|---:|
| Left Servo | 12 |
| Right Servo | 13 |

---

## Main Constants

### Servo and Driving Parameters

```cpp
const int SERVO_STOP_US = 1500;
const int LEFT_SERVO_TRIM = -10;

const int DRIVE_SPEED = 60;

const int SOFT_CORRECTION = -6;
const int STRONG_CORRECTION = -30;
```

`SERVO_STOP_US` defines the neutral pulse used to stop the servos.

`LEFT_SERVO_TRIM` compensates for a small difference between the two motors.

`DRIVE_SPEED` is the normal forward speed used while navigating corridors.

The correction values are used to steer the robot back toward the center of a corridor.

---

## Turn Calibration

```cpp
const int TURN_LEFT_90_MS = 497;
const int TURN_RIGHT_90_MS = 517;
```

The robot performs turns using timed in-place servo rotations.

The left and right turn durations are calibrated separately because the two motors and the mechanical system may not behave identically.

These values may need to be recalibrated if:

- battery voltage changes;
- wheel friction changes;
- servo behavior changes;
- the robot carries additional weight;
- the maze surface changes.

---

## IR Wall Detection

Each IR sensor is sampled using frequencies from:

```text
37 kHz to 42 kHz
```

in 1 kHz steps.

This produces six readings for each direction.

```cpp
for (long freq = 37000; freq <= 42000; freq += 1000)
```

The readings are summed to produce a sensor score.

Lower values indicate stronger reflected IR signals and therefore a higher likelihood that a wall is present.

### Wall Thresholds

```cpp
const int SIDE_WALL_LIMIT = 6;
const int FRONT_WALL_LIMIT = SIDE_WALL_LIMIT - 2;
```

Therefore:

```text
Side wall threshold  = 6
Front wall threshold = 4
```

A side is considered blocked when:

```cpp
rawValue < threshold
```

For example:

```cpp
w.blocked[LEFT_SIDE] =
    w.raw[LEFT_SIDE] < SIDE_WALL_LIMIT;
```

The front threshold is intentionally different from the side threshold so that front-wall detection can be tuned separately.

---

## Sensor Data Structure

Sensor information is grouped into the following structure:

```cpp
struct Walls {
    int raw[3];
    bool blocked[3];
};
```

For each direction, the robot stores:

- the raw IR score;
- whether that direction is considered blocked.

The direction indexes are defined by:

```cpp
enum Side {
    LEFT_SIDE = 0,
    FRONT_SIDE = 1,
    RIGHT_SIDE = 2
};
```

This avoids using unexplained numeric indexes throughout the program.

---

# Movement Control

## Forward Movement

```cpp
void driveAhead(int speed)
```

Moves both wheels forward.

---

## Reverse Movement

```cpp
void driveBack(int speed)
```

Commands both wheels in the reverse direction.

---

## Stop

```cpp
void brakeRobot()
```

Stops both servos using approximately neutral servo pulse widths.

---

## In-Place Rotation

```cpp
void spinRobot(char direction, int speed)
```

Rotates the robot in place.

Supported directions are:

```text
'L' = left
'R' = right
```

---

## 90-Degree Turn

```cpp
void quarterTurn(char direction)
```

Performs an approximately 90° turn using the calibrated timing values.

The sequence is:

```text
Start rotation
      ↓
Wait calibrated turn time
      ↓
Stop robot
      ↓
Wait 80 ms
      ↓
Reset opening counters
```

---

## Entering a New Corridor

After a turn, the robot drives forward for:

```cpp
const int AFTER_TURN_FORWARD_MS = 1051;
```

This moves the robot away from the junction and into the new corridor before normal wall detection resumes.

---

# Corridor Centering

The function:

```cpp
keepMiddleOfCorridor()
```

compares the left and right IR scores.

```cpp
int offset = leftScore - rightScore;
```

The difference is used to estimate whether the robot is closer to one side of the corridor.

### Control Logic

| Offset | Interpretation | Action |
|---:|---|---|
| `<= -3` | Too close to left wall | Strong correction right |
| `-2` | Slightly close to left wall | Soft correction right |
| `-1, 0, 1` | Approximately centered | Drive straight |
| `2` | Slightly close to right wall | Soft correction left |
| `>= 3` | Too close to right wall | Strong correction left |

This is a discrete feedback controller rather than a continuous PID controller.

---

# Side-Opening Confirmation

A single open-side reading is not immediately treated as a junction.

The program uses:

```cpp
const int OPENING_CONFIRM_LIMIT = 2;
```

An opening must therefore be detected in two consecutive sensor scans before the robot commits to the corresponding turn.

The counters are:

```cpp
int leftOpenSeen = 0;
int rightOpenSeen = 0;
```

This helps reduce false turns caused by:

- sensor noise;
- temporary IR interference;
- small gaps;
- unstable readings near wall edges.

---

# Junction Alignment

Before turning into an opening, the robot moves forward slightly so that its rotation point is better aligned with the center of the junction.

This is controlled by:

```cpp
void alignAndTurn(char direction, bool frontIsBlocked)
```

Two different alignment times are used.

### Side Opening With Open Front

```cpp
const int SIDE_OPEN_FORWARD_ALIGN_MS = 900;
```

### Front Blocked

```cpp
const int FRONT_BLOCKED_ALIGN_MS = 241;
```

The shorter value is used when the robot is already close to a front wall.

---

# T-Junction Detection

When:

```text
Front = blocked
Left  = open
Right = open
```

the robot does not immediately assume that it has reached a perfect T-junction.

Instead, it calls:

```cpp
inspectTJunction();
```

The robot creeps forward for:

```cpp
const int T_RECHECK_FORWARD_MS = 150;
```

and scans the maze again.

This second measurement helps distinguish between:

- a left corner;
- a right corner;
- a true T-junction;
- a dead end.

---

## Confirmed Left Corner

Condition:

```text
Left  = open
Front = blocked
Right = blocked
```

Action:

```text
Turn left
```

---

## Confirmed Right Corner

Condition:

```text
Left  = blocked
Front = blocked
Right = open
```

Action:

```text
Turn right
```

---

## Confirmed T-Junction

Condition:

```text
Left  = open
Front = blocked
Right = open
```

If there is no recent U-turn context, the default behavior is:

```text
Turn left
```

The robot also stores the previous turn so that the navigation decision can be adjusted after returning from a dead end.

---

# Navigation Memory

Several variables allow the robot to remember limited information about previous movement.

```cpp
char previousTurn = ' ';
bool uTurnJustMade = false;
```

`previousTurn` stores:

```text
'L' — previous important turn was left
'R' — previous important turn was right
```

When the robot performs a U-turn and later returns to a T-junction, this information is used to determine which direction should be taken.

This provides a small amount of route context without maintaining a complete representation of the maze.

---

# Dead-End Detection

A possible dead-end corridor is identified when:

```text
Left  = blocked
Right = blocked
Front = open
```

The robot begins timing how long it remains in this narrow corridor.

```cpp
possibleDeadEndStartMs = millis();
```

If the robot eventually reaches:

```text
Left  = blocked
Front = blocked
Right = blocked
```

the elapsed corridor time is examined.

---

## Short Dead End

If:

```cpp
elapsed < LONG_DEAD_END_MS
```

the area is interpreted as a normal dead end.

The robot:

```text
Stops
  ↓
Performs a U-turn
  ↓
Drives forward
  ↓
Returns toward the previous junction
```

---

## Long Dead End / Finish Detection

The threshold is:

```cpp
const unsigned long LONG_DEAD_END_MS = 1400;
```

If the robot has travelled through the narrow corridor for at least this amount of time before reaching the blocked end, it treats that location as the finish area.

```cpp
if (timingNarrowCorridor &&
    elapsed >= LONG_DEAD_END_MS) {

    brakeRobot();

    while (true) {
    }
}
```

The infinite loop deliberately prevents any further navigation after the finish is detected.

---

# U-Turn Recovery

The U-turn is performed by:

```cpp
rotateBack();
```

using:

```cpp
const int UTURN_SPIN_SPEED = 250;
const int UTURN_SPIN_MS = 980;
```

After the U-turn, the robot temporarily ignores side openings.

This is controlled by:

```cpp
bool ignoreSideEdges = false;
bool sawOpenAreaAfterUTurn = false;
```

The reason is that immediately after leaving a dead end, the robot may pass through the same junction geometry that previously caused a turn.

Without suppression, the robot could prematurely detect a side edge and make an incorrect turn before completely leaving the junction.

Side-opening detection is re-enabled after the robot has:

1. detected an open junction area; and
2. returned to a corridor with walls on both sides.

---

# Startup Protection

Immediately after startup, the robot ignores side-turn decisions for:

```cpp
const unsigned long START_IGNORE_TURNS_MS = 700;
```

During this period, if the front is open, the robot only moves forward and performs corridor balancing.

This reduces the likelihood that the initial maze position is incorrectly interpreted as a side junction.

---

# Main Navigation Algorithm

The main navigation logic is implemented in:

```cpp
void loop()
```

The decision priority is important.

```text
Scan IR sensors
       ↓
Update dead-end timer
       ↓
Are all three directions blocked?
       │
       ├── YES → resolve dead end
       │
       └── NO
            ↓
Is post-U-turn edge suppression active?
       │
       ├── YES → continue forward when possible
       │
       └── NO
            ↓
Still in startup protection?
       │
       ├── YES → move forward
       │
       └── NO
            ↓
Confirmed left opening?
       │
       ├── YES → align and turn left
       │
       └── NO
            ↓
Confirmed right opening?
       │
       ├── YES → align and turn right
       │
       └── NO
            ↓
Is front open?
       │
       ├── YES → corridor centering
       │
       └── NO
            ↓
Front blocked and both sides open?
       │
       ├── YES → inspect T-junction
       │
       └── NO
            ↓
Stop for safety
```

---

# Navigation Decision Table

| Left | Front | Right | Main Behaviour |
|---|---|---|---|
| Blocked | Open | Blocked | Continue through corridor |
| Open | Open | Blocked | Confirm opening, then turn left |
| Blocked | Open | Open | Confirm opening, then turn right |
| Open | Blocked | Blocked | Turn left |
| Blocked | Blocked | Open | Turn right |
| Open | Blocked | Open | Inspect T-junction |
| Blocked | Blocked | Blocked | Dead-end handling |

Actual behavior may also depend on:

- startup lockout;
- opening confirmation counters;
- previous turn;
- U-turn state;
- dead-end timer;
- post-U-turn edge suppression.

---

# Serial Debugging

The program outputs detailed diagnostic information at:

```cpp
Serial.begin(9600);
```

Examples include:

```text
L: 2 | M: 6 | R: 3
```

for IR measurements,

```text
Centered -> move forward
```

for corridor control,

```text
Align then turn LEFT
```

for junction handling,

and:

```text
Dead end! Time in corridor: 920 ms
SHORT dead end -> reverse then U-turn
```

for dead-end processing.

Servo pulse values are also printed:

```text
servo_1: 1450 | servo_2: 1560
```

These messages are useful when calibrating the robot or diagnosing incorrect maze decisions.

---

# Important Functions

| Function | Purpose |
|---|---|
| `writeDrive()` | Sends movement commands to both servos |
| `driveAhead()` | Drives forward |
| `driveBack()` | Drives backward |
| `brakeRobot()` | Stops both motors |
| `spinRobot()` | Rotates robot in place |
| `quarterTurn()` | Performs calibrated 90° turn |
| `rotateBack()` | Performs U-turn |
| `enterNewCorridor()` | Moves robot away from junction after turning |
| `readIRPulse()` | Reads one IR receiver at one frequency |
| `measureIR()` | Performs multi-frequency IR measurement |
| `scanMaze()` | Reads all three directions |
| `keepMiddleOfCorridor()` | Corrects corridor position |
| `refreshDeadEndTimer()` | Tracks possible dead-end corridor duration |
| `alignAndTurn()` | Aligns robot before turning |
| `resolveDeadEnd()` | Handles short and long dead ends |
| `continueAfterUTurnIfNeeded()` | Suppresses false side-edge detection |
| `inspectTJunction()` | Rechecks ambiguous T-junction geometry |

---

# Installation and Upload

## Requirements

- Arduino-compatible development environment;
- Arduino `Servo` library;
- two continuous-rotation servos;
- three IR transmitter/receiver pairs;
- suitable power supply for the motors and controller.

The Servo library is included with standard Arduino installations:

```cpp
#include <Servo.h>
```

---

## Upload Procedure

1. Connect the IR emitters, receivers, LEDs, and servos according to the hardware table.
2. Open the program in the Arduino IDE.
3. Select the appropriate board.
4. Select the appropriate serial port.
5. Compile the sketch.
6. Upload it to the robot.
7. Open the Serial Monitor.
8. Set the baud rate to:

```text
9600 baud
```

9. Place the robot at the maze starting position.
10. Reset or power on the robot.

---

# Calibration

The current implementation depends heavily on physical timing and sensor thresholds.

The following constants should be tested on the actual robot.

## IR Thresholds

```cpp
SIDE_WALL_LIMIT
FRONT_WALL_LIMIT
```

Adjust these if the robot incorrectly classifies open space as a wall or fails to detect walls.

---

## 90° Turns

```cpp
TURN_LEFT_90_MS
TURN_RIGHT_90_MS
```

Increase the corresponding value if the robot under-rotates.

Decrease it if the robot over-rotates.

---

## Junction Alignment

```cpp
SIDE_OPEN_FORWARD_ALIGN_MS
FRONT_BLOCKED_ALIGN_MS
```

These determine where the robot positions itself before turning.

Incorrect values may cause:

- clipping maze walls;
- turning too early;
- turning too late;
- entering the next corridor at an angle.

---

## Post-Turn Forward Movement

```cpp
AFTER_TURN_FORWARD_MS
```

This controls how far the robot moves into the new corridor before normal navigation resumes.

---

## U-Turn

```cpp
UTURN_SPIN_SPEED
UTURN_SPIN_MS
```

These determine the approximate 180° rotation.

---

## Dead-End Classification

```cpp
LONG_DEAD_END_MS
```

This distinguishes an ordinary short dead end from the designated long finish corridor.

Changing driving speed may require this threshold to be recalibrated.

---

# Design Considerations

## Advantages

The current design includes several mechanisms intended to improve robustness:

- multi-frequency IR sampling;
- separate front and side thresholds;
- two-sample side-opening confirmation;
- startup junction suppression;
- pre-turn alignment;
- T-junction rechecking;
- corridor centering;
- dead-end timing;
- post-U-turn edge suppression;
- limited previous-turn memory;
- safety stop when no valid state is identified.

---

## Limitations

The program is primarily a **reactive navigation controller** rather than a full maze-solving algorithm.

It does not construct a complete map of the maze and does not maintain a stack or graph containing every previous junction.

Several behaviors also depend on fixed timing constants such as:

```cpp
delay(TURN_LEFT_90_MS);
delay(SIDE_OPEN_FORWARD_ALIGN_MS);
delay(AFTER_TURN_FORWARD_MS);
```

Therefore, navigation accuracy depends on relatively stable:

- motor speed;
- battery voltage;
- maze dimensions;
- wheel traction;
- IR reflection characteristics.

The corridor controller is also based on discrete IR score differences rather than a continuously calibrated distance estimate.

---

# Possible Future Improvements

Potential extensions include:

- replace fixed delays with encoder-based movement;
- use proportional or PID corridor control;
- average or filter multiple IR measurements;
- replace blocking `delay()` calls with a non-blocking state machine;
- dynamically calibrate IR thresholds;
- maintain a stack of junction decisions;
- implement Trémaux, DFS, or another formal maze-solving algorithm;
- store a maze graph for route optimization;
- use wheel encoders or an IMU for more accurate turns;
- separate sensor, motion, and navigation logic into independent modules.

---

# Summary

This program controls an autonomous IR-based maze robot using three directional sensors and two continuous-rotation servos.

Its navigation pipeline combines:

```text
IR sensing
   +
wall classification
   +
corridor correction
   +
junction confirmation
   +
timed turning
   +
dead-end detection
   +
U-turn recovery
   +
limited navigation memory
```

The design is intended to provide reliable navigation in a known physical maze environment while remaining simple enough to execute on an Arduino-class microcontroller.

