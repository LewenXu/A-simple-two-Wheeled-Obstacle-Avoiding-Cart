# USYD ELEC1601 Maze Robot README

## 1. Project Overview

This project is an Arduino-based maze-navigation robot program.
The robot uses three infrared sensor pairs to detect walls on the left, front, and right sides, and uses two continuous-rotation servos to move through a maze.

The program supports:

* forward movement through corridors;
* left and right wall detection;
* corridor centering by comparing left and right IR sensor readings;
* left and right turn decisions at openings;
* 90-degree timed turns;
* T-junction confirmation by creeping forward and checking again;
* dead-end detection;
* U-turn behaviour at short dead ends;
* permanent stop at long dead ends, treated as the finish area;
* startup turn lockout to avoid false turns immediately after the robot starts;
* post-U-turn suppression to avoid turning back into the same dead end.

---

## 2. Hardware Used

The program assumes the following hardware components:

* Arduino-compatible board;
* two continuous-rotation servos;
* three IR LED and IR receiver pairs:

  * left sensor;
  * middle/front sensor;
  * right sensor;
* three indicator LEDs:

  * left LED;
  * middle LED;
  * right LED.

---

## 3. Pin Configuration

| Component          | Pin |
| ------------------ | --: |
| Left IR LED        |  10 |
| Left IR Receiver   |  11 |
| Middle IR LED      |   6 |
| Middle IR Receiver |   7 |
| Right IR LED       |   2 |
| Right IR Receiver  |   3 |
| Left LED           |  A2 |
| Middle LED         |  A1 |
| Right LED          |  A0 |
| Servo 1            |  12 |
| Servo 2            |  13 |

The pin definitions are placed at the top of the code so that wiring changes can be made easily.

---

## 4. Main Constants and Tuning Values

### Servo and Movement Values

| Constant                 |  Value | Purpose                                                                |
| ------------------------ | -----: | ---------------------------------------------------------------------- |
| `STOP_VALUE`             |   1500 | Neutral pulse width for stopping the servos                            |
| `CALIBRATION_VALUE_LEFT` |    -10 | Calibration offset for the left servo                                  |
| `FORWARD_SPEED`          |     60 | Normal forward speed                                                   |
| `CORRECTION_SMALL`       |     -6 | Small correction used when the robot is slightly off-centre            |
| `CORRECTION_BIG`         |    -30 | Larger correction used when the robot is clearly too close to one wall |
| `LEFT_TURN_TIME`         | 497 ms | Timed duration for a left 90-degree turn                               |
| `RIGHT_TURN_TIME`        | 517 ms | Timed duration for a right 90-degree turn                              |
| `U_TURN_SPEED`           |    250 | Speed used for a U-turn                                                |
| `U_TURN_TIME`            | 980 ms | Timed duration for a 180-degree U-turn                                 |

### Sensor Thresholds

| Constant               |                Value | Purpose                                                     |
| ---------------------- | -------------------: | ----------------------------------------------------------- |
| `WALL_THRESHOLD`       |                    6 | General wall-detection threshold for left and right sensors |
| `FRONT_WALL_THRESHOLD` | `WALL_THRESHOLD - 2` | Stricter threshold for the front sensor                     |

The sensor logic treats a lower IR distance score as a stronger wall detection. Therefore:

```cpp
s.leftWall  = s.left  < WALL_THRESHOLD;
s.frontWall = s.mid   < FRONT_WALL_THRESHOLD;
s.rightWall = s.right < WALL_THRESHOLD;
```

---

## 5. Program Structure

The program is divided into the following main sections:

1. pin definitions;
2. tuning constants;
3. global navigation state;
4. sensor state structure;
5. setup function;
6. movement functions;
7. sensor reading functions;
8. corridor-following logic;
9. dead-end tracking;
10. turn helper functions;
11. dead-end handling;
12. post-U-turn suppression;
13. T-junction handling;
14. main loop.

This structure separates low-level hardware control from high-level navigation decisions.

---

## 6. Sensor Reading Logic

The robot estimates distance by testing several IR frequencies from 37 kHz to 42 kHz.

```cpp
for (long f = 37000; f <= 42000; f += 1000) {
  distance += irDetect(irLedPin, irReceiverPin, f);
}
```

The total distance score is then used to decide whether a wall is present.

A smaller score usually means the wall is closer or more strongly detected.

The three sensor readings are stored in a `SensorState` structure:

```cpp
struct SensorState {
  int left;
  int mid;
  int right;

  bool leftWall;
  bool frontWall;
  bool rightWall;
};
```

This makes the main loop easier to read because it can use clear Boolean values such as `leftWall`, `frontWall`, and `rightWall`.

---

## 7. Movement Functions

### `setServos(int leftServoSpeed, int rightServoSpeed)`

This is the lowest-level movement function.
It converts logical speed values into microsecond PWM signals for the two servos.

```cpp
int pwm1 = STOP_VALUE - leftServoSpeed - CALIBRATION_VALUE_LEFT;
int pwm2 = STOP_VALUE + rightServoSpeed;
```

Because the two servos are mounted in opposite directions, their PWM calculations are different.

### `moveForward(int speed)`

Moves both wheels forward at the same logical speed.

### `moveBackward(int speed)`

Moves both wheels backward by passing negative speeds into `setServos`.

### `stop()`

Stops both servos by sending the neutral pulse width.

### `turnLeft90()` and `turnRight90()`

These functions perform timed 90-degree turns.
They use fixed delay values rather than feedback-based turning.

### `uTurn()`

Performs a timed 180-degree turn, mainly used when the robot reaches a short dead end.

---

## 8. Corridor Following

The robot follows a corridor by comparing the left and right sensor distance scores.

```cpp
int difference = left - right;
```

The program then adjusts the two servo speeds:

* if the robot is too close to the left wall, it steers right;
* if the robot is too close to the right wall, it steers left;
* if the difference is small, it moves forward normally.

The correction is split into two levels:

* small correction;
* large correction.

This prevents the robot from overreacting to minor sensor differences.

---

## 9. Dead-End Tracking

The function `updateDeadEndTracking()` starts timing when the robot enters a narrow corridor where:

```cpp
s.leftWall && s.rightWall && !s.frontWall
```

This means:

* left wall detected;
* right wall detected;
* front is open.

If the robot later reaches a dead end, the elapsed time in that corridor is used to decide whether the dead end is short or long.

---

## 10. Dead-End Handling

A dead end is detected when all three directions are blocked:

```cpp
s.frontWall && s.leftWall && s.rightWall
```

When this happens, `handleDeadEnd()` is called.

The code calculates:

```cpp
unsigned long timeInCorridor = millis() - corridorEntryTime;
```

If the robot has spent at least `LONG_DEAD_END_THRESHOLD_MS` in the corridor, the program treats this as the finish area and stops permanently:

```cpp
while (true) {}
```

If the dead end is short, the robot performs a U-turn and then moves forward slightly.

Important note: the constant `BACKUP_BEFORE_UTURN_MS` is defined, but the current `handleDeadEnd()` implementation does not call `moveBackward()` before the U-turn.

---

## 11. Post-U-Turn Suppression

After a U-turn, the robot may pass the same junction area again.
Without protection, it could immediately detect the same side opening and turn back into the dead end.

To prevent this, the program uses:

```cpp
suppressEdgeDetection
postUTurnOpenAreaSeen
```

While suppression is active:

* side-opening detection is temporarily ignored;
* the robot continues forward if the front is open;
* normal side-opening detection is re-enabled only after the robot has clearly returned to a corridor with walls on both sides.

---

## 12. T-Junction Handling

When the front is blocked and both sides are open, the robot may be facing a T-junction.

The program does not turn immediately.
Instead, it creeps forward for a short time:

```cpp
moveForward(80);
delay(T_JUNCTION_CREEP_MS);
```

Then it reads the sensors again.

This second reading helps distinguish between:

* left-turn corner;
* right-turn corner;
* true T-junction;
* dead end.

If it is a true T-junction, the default behaviour is to turn left unless the robot recently performed a U-turn and has a stored previous turn direction.

---

## 13. Main Loop Decision Order

The main loop reads sensors once per cycle, updates dead-end tracking, and then checks navigation cases in priority order.

The order is important:

1. dead end;
2. post-U-turn suppression;
3. startup lockout;
4. left opening;
5. right opening;
6. normal corridor;
7. T-junction;
8. safety fallback.

This prevents the robot from making unsafe or conflicting decisions.
For example, dead-end handling is checked before normal movement, and post-U-turn suppression is checked before normal side-opening detection.

---

## 14. Startup Lockout

For the first `START_TURN_LOCKOUT_MS` milliseconds after startup, side openings are ignored if the front is open.

This avoids false turns caused by unstable sensor readings immediately after the robot starts.

```cpp
if (millis() - startTime < START_TURN_LOCKOUT_MS && frontOpen) {
  followCorridor(s.left, s.right);
  return;
}
```

---

## 15. Navigation Behaviour Summary

| Situation                              | Robot Behaviour                             |
| -------------------------------------- | ------------------------------------------- |
| Front open, no side opening confirmed  | Follow corridor                             |
| Right wall detected and left side open | Confirm left opening, then turn left        |
| Left wall detected and right side open | Confirm right opening, then turn right      |
| Front blocked, both sides open         | Treat as possible T-junction and re-check   |
| Left, front, and right all blocked     | Handle as dead end                          |
| Short dead end                         | Perform U-turn                              |
| Long dead end                          | Stop permanently as finish area             |
| Immediately after U-turn               | Suppress side-opening detection temporarily |

---

## 16. Calibration Guide

If the robot does not move correctly, tune these values first.

### Robot drifts while moving forward

Adjust:

```cpp
CALIBRATION_VALUE_LEFT
```

If the robot drifts left, try increasing the value.
If the robot drifts right, try decreasing the value.

### Robot turns too far or not far enough

Adjust:

```cpp
LEFT_TURN_TIME
RIGHT_TURN_TIME
U_TURN_TIME
```

Increase the time if the robot under-turns.
Decrease the time if the robot over-turns.

### Robot detects walls incorrectly

Adjust:

```cpp
WALL_THRESHOLD
FRONT_WALL_THRESHOLD
```

If the robot detects walls that are not there, the threshold may be too high.
If the robot misses real walls, the threshold may be too low.

### Robot turns too early or too late at openings

Adjust:

```cpp
SIDE_OPEN_ALIGN_MS
CORNER_ALIGN_MS
```

Increase the alignment time if the robot turns too early.
Decrease the alignment time if the robot turns too late or misses the junction.

---

## 17. Serial Debug Output

The program prints useful debugging information to the Serial Monitor, including:

* raw left, middle, and right sensor scores;
* servo PWM values;
* wall-following correction decisions;
* detected left or right openings;
* dead-end timing;
* T-junction confirmation results;
* U-turn and post-U-turn suppression messages.

Use the Serial Monitor at:

```text
9600 baud
```

---

## 18. Limitations

This program relies on fixed timing values for turns and U-turns.
Therefore, its performance may change depending on:

* battery voltage;
* servo calibration;
* wheel friction;
* floor surface;
* robot weight distribution;
* IR reflection from wall material;
* ambient lighting;
* maze geometry.

The program does not use encoder feedback, gyroscope feedback, or continuous turn-angle correction.
For more reliable turning, additional sensors or closed-loop control would be needed.

---

## 19. How to Use

1. Connect the IR sensors, LEDs, and servos according to the pin configuration.
2. Upload the Arduino sketch to the board.
3. Open the Serial Monitor at 9600 baud.
4. Place the robot at the maze start position.
5. Turn on the robot and observe the sensor values.
6. Adjust thresholds and movement timing values as needed.
7. Test again until the robot reliably follows corridors, turns at openings, and handles dead ends.

---

## 20. Key Design Idea

The central idea of the program is:

> Read the three IR sensors, convert them into wall states, then choose one navigation action according to a fixed priority order.

The robot does not move the maze data or store a full map.
Instead, it makes local decisions based on the current left, front, and right sensor readings, plus a small amount of state such as previous turn direction, dead-end timing, and U-turn suppression.
