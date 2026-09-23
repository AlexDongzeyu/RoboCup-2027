# RoboCup 2027

## Branch policy

**`main` is frozen. Do not commit, merge, rebase, reset, or push changes to
`main` unless the repository owner explicitly authorizes it. Use `dev` for
development and integration.**

The September 23, 2026 consolidation started from a clean local checkout of
`main` at `68078e4`, which was also the original `dev` tip. Newer local editor
checkpoints had the same file tree, not additional unpublished code.

| Original branch | Preserved tip | Contents |
| --- | --- | --- |
| `main` / original `dev` | `68078e4` (April 1) | Original modular firmware, libraries, examples, and Python tools |
| `new-changes` | `e4e5db4` (April 8) | IR sampling/buffer fixes, angle filtering, compass zeroing, bounded color startup, movement updates, and IR simulation |
| `good-changes-works` | `86bd9a6` (April 10) | Earlier BallTracker rewrite and its merge history |
| `New-Changes---New-Code---Working` | `5b613ff` (April 10) | Latest BallTracker pursuit speed and smooth orbit/deadzone tuning |

All of these commits are ancestors of `dev`; consolidation used real merges,
not squashes or file replacement. Deleting the three superseded branch names
does not remove their code or history. No robot control behavior was changed
during consolidation.

## Choosing firmware

The rewrite has a separate Git history and different angle/motor conventions.
Both implementations are retained as **separate sketches**, not combined into
one control loop.

| Sketch | Purpose | Serial baud | Compile check |
| --- | --- | --- | --- |
| [BallTracker](BallTracker/BallTracker.ino) | Latest April 10 firmware: IR tracking, heading hold, motor control, and selectable color avoidance/diagnostics | 115200 | Passed |
| [Original firmware](src/main/main.ino) | April 8 modular implementation, including its IR, compass, color, and movement updates | 9600 | Passed |
| [Camera example](src/main/components/camera/camera.ino) | Standalone Pixy2 SPI object-tracking example | USB serial | Passed |

The checks above used Teensy 4.1 board support `teensy:avr@1.62.0` on Windows
on September 23, 2026. BallTracker has existing unused-variable warnings; the
camera example has an existing constructor-initialization-order warning.
Compilation does **not** establish correct wiring, sensor calibration, motor
direction, or on-field behavior. No firmware was uploaded and no motor
commands were sent. USB discovery identifies the board model, not the source
revision currently flashed onto it.

### BallTracker modes

Select `TEST_MODE` in [BallTracker.ino](BallTracker/BallTracker.ino) before
building. The preserved default is **4**, with `basePursueSpeed = 160`.

| Mode | Behavior |
| --- | --- |
| 0 | Ball tracking, compass heading hold, and white-line avoidance |
| 1 | IR angle diagnostics without drive commands |
| 2 | Cardinal-direction motor test |
| 3 | Constant east drive for tuning |
| 4 | Ball tracking and heading hold; **white-line avoidance is disabled** |
| 5 | Color-sensor diagnostics and avoidance movement |

Modes other than 1 can drive the robot. Do not treat the default ball-only
mode as a competition-ready configuration. Keep motor power disconnected
for initial USB checks and verify the selected mode and wiring before any
intentional upload or powered test. The two firmware versions retain their
own sensor and motor mappings; do not interchange those mappings blindly.

### Retained experimental code and known limitations

- The camera, kicker, and time-of-flight modules are preserved but are not
  integrated into either current ball-tracking entry point.
- The [Python visualizer](src/python/ir-emulator.py) and
  [serial reader](src/python/reading_serial_monitor.py) pass syntax checks
  but still use a hard-coded macOS port. An isolated test of the existing
  parser produced 32 values from a 16-value historical payload and raised
  `ValueError` for both current firmware output formats. They need port and
  protocol fixes before use; merely selecting a Windows COM port is not
  sufficient.
- The unused [kicker prototype](src/main/components/kicker/kicker.h) fails a
  standalone compiler syntax check because its class definition is missing
  the terminating semicolon. It is retained unchanged, not silently counted
  as a working feature.
- The [time-of-flight prototype](src/main/components/tof/tof.h) needs the
  additional Pololu `VL53L0X` library, which is not bundled. It was not
  hardware-tested.
- The explicitly scrapped [Component prototype](src/main/components/components.h)
  and its implementation remain archival, not part of the verified builds.
- Use [Libraries](Libraries) for the builds below. The rewrite's
  [Libraries copy](Libraries%20copy) directory is retained for preservation,
  not as a second library search path. Its editor configuration also retains
  the original Mac-specific compiler path; that is not the firmware toolchain.

### Compile without uploading

From the repository root, with `arduino-cli` on `PATH`:

```powershell
arduino-cli core install teensy:avr@1.62.0 --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli compile --fqbn teensy:avr:teensy41 --build-property "recipe.hooks.postbuild.1.pattern=" --libraries ".\Libraries" ".\BallTracker"
arduino-cli compile --fqbn teensy:avr:teensy41 --build-property "recipe.hooks.postbuild.1.pattern=" --libraries ".\Libraries" ".\src\main"
arduino-cli compile --fqbn teensy:avr:teensy41 --build-property "recipe.hooks.postbuild.1.pattern=" --libraries ".\Libraries" ".\src\main\components\camera"
```

The empty post-build hook prevents the Teensy Loader from being launched by
these compile-only checks. It does not modify the installed board package.
Arduino IDE also bundles `arduino-cli.exe` under
`resources\app\lib\backend\resources` in its installation directory.
See [upload instructions](How%20To%20Upload%20Code.txt) before an intentional
hardware upload.

## What is RoboCup Junior?

RoboCup Junior is an international robotics competition that encourages students to design, build, and program autonomous robots to complete specific challenges. In the Soccer division, robots must detect, chase, and kick a ball into a goal while avoiding opponents and staying within the field boundaries. It’s a fast-paced, dynamic event that tests engineering, coding, and problem-solving skills under real-time constraints.

This repository contains the codebase for our custom-built soccer-playing robot,
including the original 2025/2026 components and the later BallTracker rewrite.
The component descriptions below are a reference to retained code, not a claim
that every component is enabled in the current firmware.

## Overview

The robot is programmed using Arduino (C++) for embedded control and Python for simulation and debugging. The architecture is modular to allow for fast iteration and easier testing of individual components like movement, vision, and sensors.

## Hardware Components

### 1. Movement

Controls the robot’s motion using four omnidirectional motors in an X configuration.
	•	Motors: Each wheel has two control pins for direction.
	•	Compass: Ensures the robot maintains orientation.
	•	Color Sensor: Detects the field’s white boundary to prevent out-of-bounds errors.

#### Key functions:
	•	move(theta, maxSpeed, avoid, cameraRotationAngle)
	•	brake()
	•	rotate(speed)
	•	basic_move_with_compass_and_camera(theta, maxSpeed, camAngle)


### 2. IR Sensor

Tracks the angle of the ball using a ring of 16 IR receivers mounted under the robot.

#### Key functions:
	•	getBallAngle()
	•	getReadingsArr()
	•	initIR()



### 3. Color Sensor

Reads field surface color to detect white boundary lines.

#### Key functions:
	•	isDetected()
	•	countFront(), countRight(), countBack(), countLeft()
	•	printReadings()



### 4. Camera

Uses a Pixy2 camera over SPI to track visual objects like the goal.

#### Key functions:
	•	initialize()
	•	calculateRotationAngle()
	•	findDistance()
	•	printStatus()



### 5. Compass

An Adafruit BNO055 IMU is used for absolute orientation.

#### Key functions:
	•	initialize()
	•	readCompass()



### 6. Kicker

Triggers a solenoid or mechanical mechanism to kick the ball.

Key functions:
	•	performKick()
	•	triggerKick()



#### 7. Time-of-Flight (TOF) Sensor

Uses a VL53L0X laser distance sensor to detect proximity (e.g., ball distance).

Key functions:
	•	initialize()
	•	GetBallRange()



### 8. Python Tools

Used for debugging and visualization.
	•	ir-emulator.py: Simulates IR input using Pygame.
	•	reading_serial_monitor.py: Parses serial data from the Arduino.



## How to Use

### Setup
	1.	Wire all components according to the wiring diagram (see src/main).
	2.	Choose one sketch from the firmware table above and verify its mode and hardware mapping before intentionally uploading it to the Teensy.
	3.	Use Serial Monitor at that sketch's baud rate. The retained Python visualizer needs the fixes described above before it can consume the current output.

### Initialization (setup())
	•	The ball-tracking sketches initialize IR, color sensors, compass, and motors. Camera, kicker, and time-of-flight are not initialized by these entry points.

### Main Loop (loop())
	1.	Use IR to locate the ball.
	2.	Use compass heading correction while moving toward or orbiting the ball.
	3.	Apply white-line avoidance in the original firmware or BallTracker mode 0; BallTracker mode 4 deliberately ignores it.
	4.	The camera and kicker remain separate modules, not active steps in these loops.



## File Structure
```
BallTracker/
  BallTracker.ino
  ColorSensor.cpp / ColorSensor.h
  IRRing.cpp / IRRing.h
  RobotCompass.cpp / RobotCompass.h
  RobotMotors.cpp / RobotMotors.h
Libraries/
Libraries copy/
src/
  main/
    main.ino
    components/
      camera/
        camera.h
        camera.ino
      colorsensor/
        colorsensor.cpp
        colorsensor.h
      compass/
        compass.cpp
        compass.h
      IR/
        IR.cpp
        IR.h
      kicker/
        kicker.h
      motor/
        motor.cpp
        motor.h
      movement/
        movement.cpp
        movement.h
      tof/
        tof.cpp
        tof.h
  python/
    ir-emulator.py
    reading_serial_monitor.py
```



## License

This project is licensed under the MIT License. See the LICENSE file for details.



## Contributors
  - Luca Wang
  - Rudraa Manjrekar



## Future Improvements
	•	Improve object recognition and opponent tracking with the Pixy2.
	•	Add kicker and dribbler to improve performance.
	•	Develop a visual dashboard for real-time game data.



## Contact

For questions, contact Luca Wang at lucawang626@gmail.com
