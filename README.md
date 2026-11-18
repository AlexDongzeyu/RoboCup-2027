# RoboCup Junior Soccer — 2026–2027 Robot & Firmware (`RoboCup 2027 NEW`)

Autonomous omnidirectional soccer robot built for **RoboCup Junior Soccer (Germany 2026–2027 season)**, running on a **Teensy 4.1** with four custom PCBs and a 3D-printed two-tier chassis.

This repository is the **clean-slate 2026–2027 firmware codebase**, replacing the multi-branch 2025–2026 repository. It implements **`IR11-PURSUIT`**: detect the **uncovered** pulsed IR ball across 360° (from `> 2 m` in reflective indoor rooms down to full close-range `15/16`-sensor saturation), snap the front concave ball holder (`0°` / `"凹"`) onto the ball with breakaway-compensated torque, sprint at full 4-wheel power (`255/255` PWM), and push the captured ball continuously into the goal.

> **Current Hardware/Firmware Status**
> - **Local source (`IR11-PURSUIT`):** Compiled cleanly for Teensy 4.1 (`-Wall -Wextra -Wshadow -Werror`, `0` heap allocations), verified directly in native C++17 with UBSan (`-fsanitize=undefined`) against **all 40 real-hardware `COM9` `RawIR` captures** from this physical robot (`40/40` uncovered room-bounce frames detected and driving motors, `45,040` verified MDD3A pin writes, `0` errors) plus 360° closed-loop 2D rigid-body pursuit/push simulations. **Ready to upload to the physical robot.**
> - **Boot & Calibration Behavior:** Automatically loads any saved EEPROM holder zero (`Cal:SAVED`, e.g. `357.4°`); if EEPROM is blank (`Cal:DEFAULT`) or corrupt (`Cal:FALLBACK`), safely defaults to `0.0°` (`S1` front sensor, physically centered above the `"凹"` holder) instead of locking the motors out.
> - **Active vs. Installed Hardware:** Only the **16-sensor IR ring** and **4-wheel omni drivetrain** are active in `IR11-PURSUIT`. All other onboard electronics (compass, 8 floor colour sensors, Pixy2 camera, solenoid kicker, OLED) are physically wired into the robot's PCB stack and documented below for upcoming integration.

### Why Earlier Builds (`IR9-FOLLOW` / `IR10-PURSUIT`) Failed on Hardware & What `IR11-PURSUIT` Fixes

| Symptom Observed on Physical Robot | Root Cause Proven on Hardware & Real `COM9` `RawIR` Logs | Fix in `IR11-PURSUIT` |
| :--- | :--- | :--- |
| **Uncovered ball showed `Angle:none` unless an object/hand was placed in between** | The 42-LED Rovotas v2 ball emits a `1.2 kHz` 4-step staircase whose Step-1 (`Full`) pulse bounces off indoor walls/floors/tables onto **10–16 sensors at `~0.30–0.35` duty**, while **2–4 sensors sit in internal pillar/wire shadows (`0.00–0.23`)**. `IR10`'s `sorted[3..4]` picked the shadowed pins (`~0.22`), leaving `+0.10` room bounce on 12 sensors across 360° and triggering `AMBIGUOUS_DIRECTION` (`oppScore >= 0.80 * bestScore`) unless a hand/obstacle blocked the room bounce. | Restores **median background (`sorted[7..8]`)** for `<= 0.92` duty (stripping the 8-sensor Step-1 room-bounce floor cleanly across `40/40` real `COM9` captures), switches to a **shadow-complement solver** when `>= 9` sensors saturate above `0.92`, and removes false reflection-ambiguity gates (`MIN_CONTRAST = 0.004`, `MIN_COHERENCE = 0.12`). |
| **Stopped or went blind close to an uncovered ball** | At `< 35 cm`, `TSSP4038` receivers bridge 11/12 off-gaps (`rawDuty = 0.58–0.79` with `edges = 1`) or saturate continuously `LOW` (`rawDuty = 0.80–1.00`, `edges = 0`). `IR9` dropped `< 4` edges and `IR10` dropped `edges == 1` (`0.58–0.79` gap bridging). | Accepts **any pulsing sensor (`edges >= 1 && rawDuty >= 0.008`)** *plus* **saturated sensors (`rawDuty >= 0.65 && activeSensors >= 2`)**, while still rejecting an isolated single shorted-to-GND pin (`SATURATED_OR_STUCK_LOW`). |
| **Nothing moved at all ("like no code in robot")** | (1) Continuous `Angle:none` or alternating split-post shadows (`+41° / -46°` single-frame hard snaps) flipped `turn` sign to `0 PWM`; (2) sending legacy `'g'` ("go") or stray serial bytes called `stopTracking()`, latching `running = false` across USB unplug when `BOARD` stayed ON; (3) unnecessary `analogWriteFrequency()` rewrites touched shared FlexPWM submodules right before `CLDOK`. | (1) Requires **2 consecutive `> 65°` jump frames** before snapping so post-shadow jitter averages smoothly forward, and retains `count == 1` across `< 120 ms` single-frame drops; (2) **`r` and `g` both start tracking**, stray serial bytes never kill `running`, and failed `z` resumes previous zero if already running; (3) clean `analogWriteResolution(8); stop();` uses Teensy's native `4,482.29 Hz` FlexPWM init. |
| **Did not turn toward the ball / drifted off-angle** | `IR9`'s `5–39 PWM` turn gain and `0.25` drive-turn ratio fell below the **`~52 PWM` static breakaway friction** of 4 omni wheels. | In-place `Aligning` (`>= 16°` from rest) starts immediately at **`58–62 PWM`**, adds `+14 PWM` anti-stall boost if progress `< 3° / 120 ms`, aligns within **`<= 12°`**, then sprints (`DRIVE_TURN_RATIO = 0.55`, staying in `Chasing` up to **`36°`**). |
| **Stopped instead of keep pushing the ball forward** | Inside the `"凹"` recess (`< 3 cm`), the `Ø130 mm` upper IR PCB rim shades the `42 mm` golf ball while rear sensors see weak (`< 0.25`) wall bounce. | Close front capture (`strength >= 0.30` within `±28°`) arms a **`2500 ms` full-power (`255 PWM`) straight push**, ignoring weak (`< 0.28`) rear bounce while steering in `20 ms` if a nearby (`>= 0.28`) ball slips out. |

---

## Contents

1. [Subsystem Overview (Installed vs. Active)](#1-subsystem-overview-installed-vs-active)
2. [What the Robot Looks Like (Physical & Mechanical Design)](#2-what-the-robot-looks-like-physical--mechanical-design)
3. [Power Distribution & Switches](#3-power-distribution--switches)
4. [Teensy 4.1 Pinout & Custom PCBs](#4-teensy-41-pinout--custom-pcbs)
5. [Firmware Architecture (`IR11-PURSUIT`)](#5-firmware-architecture-ir11-pursuit)
6. [Upload, Calibration & Troubleshooting](#6-upload-calibration--troubleshooting)
7. [Legacy Codebase Differences & External Links](#7-legacy-codebase-differences--external-links)

---

## 1. Subsystem Overview (Installed vs. Active)

| Subsystem | Hardware on Robot | Interface | Status in `IR11-PURSUIT` |
| :--- | :--- | :--- | :--- |
| **Controller** | PJRC **Teensy 4.1** (ARM Cortex-M7, 600 MHz) | USB Serial (`115200` baud) + EEPROM | **Active** |
| **IR Ball Ring** | `16×` Vishay **TSSP4038** 38 kHz IR receivers (`22.5°` spacing) + 3D-printed 16-slot radial focuser | 16 digital inputs (`INPUT_PULLUP`, active-`LOW`) | **Active** (tracks Rovotas pulsed IR golf ball v2, `40 kHz` carrier / `1.2 kHz` staircase) |
| **Drivetrain** | `4×` DC gearmotors + double-row omni wheels (`X-drive`) + `2×` **Cytron MDD3A** dual H-bridges | 8 PWM outputs (Pins `1–8`, `4.48 kHz` dual-PWM) | **Active** |
| **Power Stack** | **3S 11.1V LiPo** (`XT60`), custom **Power Board**, `2×` **LM2596** buck converters (`5V`, `3.3V`), 3 switches | Independent `BOARD`, `MOTOR`, and `OLED` switches | **Active** (hardware power distribution) |
| **IMU / Compass** | **Adafruit BNO055** 9-DOF absolute orientation sensor (mounted chips-up on `F.Cu`, `180°` yaw offset) | `Wire1` I2C (`SCL1=16`, `SDA1=17`), addr `0x28` | Installed on PCB — *not used in `IR11-PURSUIT`* |
| **Line Sensors** | Custom **Colour Board** (**TCA9548A** mux) + `8×` **TCS34725** RGB/Clear floor sensors (`45°` spacing) | `Wire1` I2C (`0x70` mux → `CH0–CH7` at `0x29`) | Installed on PCB — *not used in `IR11-PURSUIT`* |
| **Vision Camera** | **Pixy2** smart vision camera in 3D-printed tilt mount (`PixyCam2 case`) | `SPI0` (`CS=10`, `MOSI=11`, `MISO=12`, `SCK=13`) | Installed on PCB — *not used in `IR11-PURSUIT`* |
| **Ball Kicker** | **JF-1253B** 12V 450 mA push-pull solenoid + **400W Dual-MOSFET** PWM trigger module | PWM Pins `36`, `37` + switched `11.1V` | Mechanical/PCB ready (solenoid unordered in BOM) — *not used in `IR11-PURSUIT`* |
| **Display** | **0.96" 128×64 OLED** screen | `Wire1` I2C (`SCL1=16`, `SDA1=17`) + `OLED` switch | Installed on PCB — *not used in `IR11-PURSUIT`* |

---

## 2. What the Robot Looks Like (Physical & Mechanical Design)

The robot is a compact cylindrical two-tier assembly (**Top Assembly** + **Bottom Assembly**) with an arched top carry handle, a 360° collimated IR sensor ring near the top, a central electronics tower with front-accessible **`BOARD`** and **`MOTOR`** rocker switches, and a deep **front concave ball-receiving bay ("凹" ball holder)** at floor level (`0°`).

- **3D CAD (Onshape — `Germany 2026-27`):** [Open Main Assembly in Onshape](https://cad.onshape.com/documents/d6a9c24e179e5b10c2ce2f60/w/c426bb7fe9983fc80df2e79b/e/4f4dadfa8db7efcaffd046fc)

### Top-Down Geometry & Angle Convention (Looking Down from Above)

All angles in firmware are **clockwise from the front recessed ball holder (`0°`)**:

```text
                        0° FRONT (NORTH)
                 ┌──────╲              ╱──────┐
                 │       ╲  Recessed  ╱       │  <-- Concave Ball Holder ("凹")
            315° │   [BOARD] [MOTOR] Switches │ 45°   + Kicker Housing behind recess
          ┌──────┴────────────────────────────┴──────┐
          │   M4 (FL)                      M1 (FR)   │
          │    ╲ 310°                      50° ╱     │  <-- 4× Omni Wheels in X-Drive
          │     ╲      ┌────────────────┐     ╱      │      (±40° from lateral 90°/270° axis)
   270°   │ S5         │  Central Tower │        S13 │  90°
   LEFT   │(Pin 19)    │  Teensy / PCBs │   (Pin 27) │  RIGHT
  (WEST)  │     ╱      └────────────────┘     ╲      │  (EAST)
          │    ╱ 230°                     130° ╲     │
          │   M3 (BL)                      M2 (BR)   │
          └──────┬────────────────────────────┬──────┘
            225° │   [ 3S 11.1V LiPo Bay ]    │ 135°
                 └──────────────┬─────────────┘
                           S9 (Pin 41)
                        180° REAR (SOUTH)
```

- **Concave Ball Holder (`0°`):** Semicircular `"凹"` cutout (`~7 cm` wide mouth) in the front of the bottom skirt (`~65 mm` tall) that cradles the `42 mm` golf-sized IR ball while pushing it toward the goal. Directly above this cutout sit the **`BOARD`** (left) and **`MOTOR`** (right) rocker switches and the **Teensy 4.1 Micro-USB port** facing straight forward (`0°`). Note that because the `Ø130 mm` IR PCB sits at `~95 mm` height, a `42 mm` ball tucked flush inside the `"凹"` recess enters a `< 3 cm` geometric overhang blind spot below `S1`.
- **Left/Right Mid-Deck Regulators & Bottom Sensors:** Two blue **LM2596** buck converter modules with 3-digit LED voltmeters (`5.0V` and `3.3V`) sit exposed on the left (`270°`) and right (`90°`) mid-decks. Underneath the baseplate, `8×` **TCS34725** colour sensor breakouts are screwed face-down at `45°` intervals around the perimeter, surrounding the 4 radially inward silver DC gearmotors.
- **X-Drive Wheel Angles & Wiring Mapping:** Four double-row omni wheels sit in perimeter cutouts at **±40° from the left/right lateral axis** (`50°`, `130°`, `230°`, `310°`). Empirically verified on the assembled robot (`BallTracker` cardinal test + `movement.cpp`), channels `M1` (Pins `1,2`) and `M4` (Pins `7,8`) drive the **left-side wheel pair** (`F + T`), while `M2` (Pins `4,3`) and `M3` (Pins `5,6`) drive the **right-side wheel pair** (`-F + T`).
- **Rear Battery Bay (`180°`):** Houses the 3S `11.1V` LiPo pack (`XT60` + white balance plug) at the back of the chassis.

### Vertical 4-Level Stack (Top to Bottom)

```text
  [ Level 4: Top ]     Arched Carry Handle
                       3D-Printed IR Ball Focuser + Slot Cover (16 radial collimator baffles)
  ───────────────────────────────────────────────────────────────────────────────────────────
  [ Level 3: Upper ]   IR Board (Ø130 mm circular PCB)
                         • Top side (F.Cu):    16× TSSP4038 IR sensors, BNO055 IMU, Pixy2 SPI header
                         • Bottom side (B.Cu): Teensy 4.1 (inverted) + 11 downward JST-XH connectors
  ───────────────────────────────────────────────────────────────────────────────────────────
  [ Level 2: Middle ]  Central Structural Tower (embossed "BOARD" and "MOTOR" switch cutouts on front)
                         • Power Board (85 × 57 mm) + 2× LM2596 buck converters (5V & 3.3V)
                         • 2× Cytron MDD3A dual motor drivers (stacked) + 400W MOSFET kicker module
                         • Pixy2 Camera in ventilated tilt mount + 0.96" 128×64 OLED mount
  ───────────────────────────────────────────────────────────────────────────────────────────
  [ Level 1: Bottom ]  Bottom Frame (circular baseplate with 0° ball recess & 4 wheel cutouts)
                         • Colour Board (50 × 40 mm) mounted horizontally across M1–M4 motor mounts
                         • 8× TCS34725 floor colour sensors facing down every 45° around perimeter
                         • 4× DC gearmotors (each fitted with a Ø25 mm Motor Tab Board) + omni wheels
                         • Kicker solenoid housing behind the 0° ball recess
```

---

## 3. Power Distribution & Switches

The custom **Power Board** splits the **3S 11.1V LiPo (`XT60`)** into two independently switched ground-return domains (`SW1` and `SW2`), plus an OLED power switch (`SW3`) on the IR Board:

```text
3S 11.1V LiPo Battery (XT60)
 │
 ├──► [ MOTOR Switch ] (Front-Right rocker, switches 11.1v-(2) return)
 │      ├──► MPWR1 (XT30) ──► Cytron MDD3A #1 (2 motor channels)
 │      ├──► MPWR2 (XT30) ──► Cytron MDD3A #2 (2 motor channels)
 │      └──► KCKR PWR (XT30) ► 400W Dual-MOSFET Module ──► 12V Solenoid Kicker
 │
 └──► [ BOARD Switch ] (Front-Left rocker, switches 11.1V-(3) return)
        ├──► LM2596 Buck #1 (5.0V)
        │      └──► IRBOARD 5V PWR (JST-XH) ──► Teensy 4.1 Vin (5V) + Pixy2 Camera (5V)
        │              └──► Teensy internal 3.3V reg ──► 16× TSSP4038 IR Sensors + BNO055 IMU
        │
        └──► LM2596 Buck #2 (3.3V)
               ├──► CLR SNSR BOARD PWR (JST-XH) ──► Colour Board (TCA9548A + 8× TCS34725)
               └──► IRBOARD 3.3V PWR (JST-XH) ──► [ OLED Switch ] ──► 0.96" OLED Display
```

### Critical Power & USB Rules
1. **USB powers Teensy + IR Ring + IMU even when both switches are OFF:** The 16 `TSSP4038` receivers and `BNO055` are tied to the Teensy's onboard `3.3V` output pin (`3.3V_1`), so USB alone powers the controller and IR ring for safe motor-OFF calibration (`z`) and angle verification.
2. **USB does NOT power Motors or the Colour Board (`0x70`):**
   - Motors require **Battery + `MOTOR` switch ON**.
   - The Colour Board (`TCA9548A @ 0x70`) is powered by the `3.3V` LM2596 buck, which requires **Battery + `BOARD` switch ON** (on USB alone, I2C scans on `Wire1` will report `0x70` missing).
3. **Auto-start hazard:** With a valid saved calibration and default `START_PAUSED=0`, the Teensy enables motor commands **2 seconds after power-on or upload**. Keep the **`MOTOR` switch OFF** while uploading or calibrating, and never test powered movement on a tabletop.

---

## 4. Teensy 4.1 Pinout & Custom PCBs

### Master Teensy 4.1 Pin Allocation (Verified from `IR Board.kicad_pcb`)

| Teensy Pin(s) | PCB Net / Label | Connected Hardware | Notes |
| :--- | :--- | :--- | :--- |
| **1, 2** | `M1(1)`, `M1(2)` → `B\|M1\|A` | **Motor M1** (MDD3A ch.) | Firmware: `inputA = 1`, `inputB = 2`, `polarity = -1` |
| **3, 4** | `M2(1)`, `M2(2)` → `B\|M2\|A` | **Motor M2** (MDD3A ch.) | Firmware swaps A/B: `inputA = 4`, `inputB = 3`, `polarity = -1` |
| **5, 6** | `M3(1)`, `M3(2)` → `B\|M3\|A` | **Motor M3** (MDD3A ch.) | Firmware: `inputA = 5`, `inputB = 6`, `polarity = +1` |
| **7, 8** | `M4(1)`, `M4(2)` → `B\|M4\|A` | **Motor M4** (MDD3A ch.) | Firmware: `inputA = 7`, `inputB = 8`, `polarity = +1` |
| **10, 11, 12, 13** | `CS`, `MOSI`, `MISO`, `SCK` | **Pixy2 Camera** (`SPI0`) | Powered from `5Vin` |
| **16, 17** | `SCL_Pullup`, `SDA_Pullup` | **`Wire1` I2C Bus** (`SCL1=16`, `SDA1=17`) | Shared by **BNO055** (`0x28`), **TCA9548A** (`0x70`), **OLED**. Keep total pull-up resistance `2–10 kΩ`. |
| **14–15, 18–27, 38–41** | `S1`–`S16` | **16× TSSP4038 IR Ring** | Active-`LOW` digital inputs (`INPUT_PULLUP`); see full ring table below |
| **36, 37** | `KickerPWM1`, `KickerPWM2` | **Kicker MOSFET Module** | 2-pin `KICKER PWM` JST-XH header |
| **Vin, 3.3V, GND** | `5Vin`, `3.3Vin`, `5GND`, `3.3GND` | Power rails | `5Vin` from Power Board; `3.3V` feeds IR ring + BNO055 |

### 16-Sensor IR Ring Mapping (`IR::pins[16]`)

On the PCB silkscreen, `S1`–`S16` are numbered **counter-clockwise** when viewed from the top. Firmware array `IR::pins[16]` steps **clockwise** from `S1` (`0°` Front) in `22.5°` increments:

| Software Index | Clockwise Angle | Direction | PCB Label | Teensy Pin |
| :---: | :---: | :--- | :---: | :---: |
| `0` | **0.0°** | **Front / Ball Holder (N)** | `S1` | **23** |
| `1` | 22.5° | Front-Right | `S16` | **24** |
| `2` | 45.0° | Front-Right | `S15` | **25** |
| `3` | 67.5° | Front-Right | `S14` | **26** |
| `4` | **90.0°** | **Right (E)** | `S13` | **27** |
| `5` | 112.5° | Back-Right | `S12` | **38** |
| `6` | 135.0° | Back-Right | `S11` | **39** |
| `7` | 157.5° | Back-Right | `S10` | **40** |
| `8` | **180.0°** | **Rear (S)** | `S9` | **41** |
| `9` | 202.5° | Back-Left | `S8` | **14** |
| `10` | 225.0° | Back-Left | `S7` | **15** |
| `11` | 247.5° | Back-Left | `S6` | **18** |
| `12` | **270.0°** | **Left (W)** | `S5` | **19** |
| `13` | 292.5° | Front-Left | `S4` | **20** |
| `14` | 315.0° | Front-Left | `S3` | **21** |
| `15` | 337.5° | Front-Left | `S2` | **22** |

*Note:* Holder calibration (`z`) subtracts a measured angular offset (`zeroDegrees`) so `0.0°` aligns with the exact centerline of the physical ball recess, even if `S1` is slightly offset mechanically. Calibration cannot fix a reversed left/right wiring order.

### 8-Channel Colour Sensor Ring (Hardware Reference for Boundary Avoidance)

Connected via **Colour Board** (`TCA9548A` at `0x70` on `Wire1`, address pins `A0=A1=A2=GND`; each `TCS34725` has fixed I2C address `0x29` on mux branch `CS0–CS7`):

| Mux Channel | Clockwise Angle | Position | Escape / Backup Heading |
| :---: | :---: | :--- | :---: |
| `CS0` (`CH0`) | `0°` | Front | `180°` |
| `CS1` (`CH1`) | `45°` | Front-Right | `225°` |
| `CS2` (`CH2`) | `90°` | Right | `270°` |
| `CS3` (`CH3`) | `135°` | Back-Right | `315°` |
| `CS4` (`CH4`) | `180°` | Rear | `0°` |
| `CS5` (`CH5`) | `225°` | Back-Left | `45°` |
| `CS6` (`CH6`) | `270°` | Left | `90°` |
| `CS7` (`CH7`) | `315°` | Front-Left | `135°` |

*Note:* Geometric escape headings (`sensor_angle + 180°`) are listed above; verify actual `CS0–CS7` plug order on the Colour Board when enabling boundary avoidance.

### Summary of the 4 Custom PCBs (KiCad 9, 2-Layer `1.6 mm` FR4)

1. **IR Board (`Ø130 mm` circle):** Top carrier for `16× TSSP4038` (`r = 61 mm`), `BNO055`, and `Pixy2` SPI header; bottom socket for `Teensy 4.1` and 11 `JST-XH` 2-pin subsystem headers.
2. **Power Board (`85 × 57 mm` with mounting ears):** Distributes `XT60` 3S LiPo power through `XT30` switch loops (`BOARD`, `MOTOR`) and solder pads for `2× LM2596` buck modules to `3× XT30` high-current outputs (`MPWR1`, `MPWR2`, `KCKR PWR`) and `3× JST-XH` logic outputs (`5V`, `2× 3.3V`).
3. **Colour Board (`50 × 40 mm`):** Hosts the `TCA9548A` I2C mux (`F.Cu`) + `DATA`/`3.3Vin` inputs, breaking out to `8×` downward `JST-XH` 4-pin sensor headers (`CS0–CS7` on `B.Cu`).
4. **Motor Tab Board (`Ø25 mm` circle, `4×`):** Solders directly onto each DC motor's rear `+`/`-` tabs (`2.0 mm` traces) and provides a strain-relieved 2-pin `JST` header.

---

## 5. Firmware Architecture (`IR11-PURSUIT`)

### Repository Layout

```text
RoboCup 2027 NEW/
├── README.md                        # Complete hardware, wiring, firmware & operating guide
└── src/
    └── main/
        ├── main.ino                 # Setup, s/r/g/z/t serial commands, 20 ms loop & telemetry
        └── src/
            ├── IR/
            │   ├── IR.h             # BallReading struct, thresholds & IR ring interface
            │   └── IR.cpp           # 10 ms sampler, median+shadow-complement bearing & EEPROM zero
            ├── movement/
            │   └── movement.h       # Align-then-sprint controller, stiction/PD turn & 2.5 s capture push
            └── motor/
                ├── motor.h          # Speed/ramp/ratio constants & Motors interface
                └── motor.cpp        # 4.48 kHz dual-PWM MDD3A output, ramps & 4-wheel omni mixer
```

### Control Loop Pipeline (`20 ms` / `50 Hz`)

#### 1. IR Sampling, Uncovered Room-Bounce Removal & Flood-Proof Bearing — [`IR.h`](./src/main/src/IR/IR.h) / [`IR.cpp`](./src/main/src/IR/IR.cpp)
- **`10,000 µs` (`10 ms`) simultaneous poll window:** Samples all 16 `TSSP4038` active-`LOW` pins in lockstep across **`~12` full cycles of the `1.2 kHz` (`833 µs`) RCJ staircase modulation** (and `6` cycles of `600 Hz`), capturing `~48,000` reads per pin on Teensy 4.1.
- **Full-spectrum pulse + saturation acceptance:** Accepts channel `i` whenever `rawDuty[i] >= 0.008 && edges[i] >= 1` (covering weak distant hits, normal `24`-edge modulation, and `0.58–0.99` close-range partial off-gap bridging) **or** `rawDuty[i] >= 0.65` with `activeSensors >= 2` (continuous-`LOW` close-range saturation). An isolated DC-stuck-`LOW` pin (`edges == 0 && rawDuty >= 0.80 && activeSensors == 1`) is flagged as `SATURATED_OR_STUCK_LOW`.
- **Two-regime background & saturation solver (uncovered-ball safe):**
  - **Normal & reflective indoor rooms (`medianBg <= 0.92`):** Uses **median background** `(sorted[7] + sorted[8]) / 2` to strip the `~0.30–0.35` Step-1 room-bounce pedestal cleanly even when `2–4` sensors are shadowed by internal chassis posts/wires (`MIN_CONTRAST = 0.004`, `MIN_COHERENCE = 0.12`).
  - **Wide close-range flood (`medianBg > 0.92`, `9–15` sensors saturated near `1.000`):** Switches automatically to a **capped shadow-complement vector** (`w = min(0.35, peak - signal[i])` pointing `-cos/-sin`), finding the ball `180°` opposite the rear shadow arc even when 15 of 16 sensors read `1.000`, while still rejecting an all-equal `16/16` dead `3.3V` rail as `UNIFORM_SIGNAL`.
- **Plateau-safe lobe center & `±112.5°` vector bearing:** Applies a symmetric `±5`-sensor cosine bell filter `cos(offset * π / 12)` to locate the illuminated hemisphere `bestCenter`, sums `excess^2` across `bestCenter ± 5` (`±112.5°`) so rear wall bounce cannot cancel the front vector, and rejects only virtually identical artificial twin sources (`oppScore >= 0.95 * bestScore && ringCoherence < 0.06` → `AMBIGUOUS_DIRECTION`).
- **Holder zero & fault-tolerant boot:** Subtracts `zeroDegrees` (saved in the **final 12 bytes of EEPROM** with magic `0x4952325A` and XOR checksum — **do not reuse those final 12 EEPROM bytes**). If EEPROM is blank (`Cal:DEFAULT`) or corrupt (`Cal:FALLBACK`), defaults `zeroDegrees = 0.0°` (`S1` front sensor) so the robot always runs out-of-the-box without requiring a laptop.

#### 2. Align-Then-Sprint & Continuous Push Controller — [`movement.h`](./src/main/src/movement/movement.h)
- **Glitch-tolerant acquisition & post-shadow smoothing:** Confirms a target on the **2nd valid frame (`20 ms`)**, retains `count == 1` across a single `< 120 ms` missed frame so flickering far-range acquisition never stalls, and filters unit vectors (`45%` history / `55%` new). Requires **2 consecutive `> 65°` jump frames** before snapping so alternating split-post shadows average smoothly forward instead of ping-ponging `turn` sign to `0 PWM`.
- **Asymmetric `16° / 12° / 36°` align-then-sprint hysteresis:**
  - From rest (or after a confirmed jump), any ball at `>= 16°` enters **`Aligning`** (`forward = 0`), rotating on the spot until the `"凹"` holder is within **`<= 12°`** of the ball before launching forward — preventing bumper-shoulder misses.
  - Once **`Chasing`** is active, stays in `Chasing` all the way up to **`36°`** so close-range geometric angle magnification right at the mouth of the `"凹"` recess never aborts forward pursuit.
- **Breakaway-compensated PD turning + anti-stall boost:**
  - Deadband stops turning within **`±3°`**, restarts at **`±5°`**, clears on a `0°` front crossing, and locks turn sign above `150°` so `179°/181°` rear targets never chatter.
  - **`Aligning` (`forward = 0`, overcoming static friction):** Starts at `62 + 0.42 * (|err| - 3°)` PWM (above the `~52 PWM` omni-wheel stiction floor, up to `120 PWM`), applies derivative braking `-3.2 * closingSpeed` only inside `35°` so large `90°–180°` spins stay fast, and adds `+14 PWM` (`stallBoost`, up to `+48 PWM`) every `6` frames (`120 ms`) if heading progress `< 3°` on grippy carpet.
  - **`Chasing` (`forward > 0`, kinetic rolling friction):** Smoothly steers with `24 + 2.4 * (|err| - 3°) - 2.5 * closingSpeed` (`16–95 PWM` differential) while advancing at **`255/255` PWM within `±12°`** (`135–255 PWM` across `12°–36°`).
- **Continuous close-capture goal push (`2500 ms` `PUSH_HOLD`):**
  - Approaching within `±28°` with close-range IR (`strength >= 0.30`) arms `closeCaptured`: if the ball tucks into the `< 3 cm` overhang shadow inside the `"凹"` cradle, the robot **keeps pushing straight ahead at `255/255` PWM for `2.5 seconds`** (`> 2.5 m`, enough to cross the entire field into the net) and ignores weak (`< 0.28`) rear/side wall-bounce ghosts.
  - If the ball is knocked out of the holder (`strength >= 0.28` off-axis), `IR11-PURSUIT` overrides `PUSH_HOLD` in `20 ms` (`1` frame) and turns immediately toward it. Far-range signal loss (`!closeCaptured`) stops within **`220 ms`** (`180 ms` during a turn).

#### 3. Ramping, 4-Wheel Mixing & MDD3A Output — [`motor.h`](./src/main/src/motor/motor.h) / [`motor.cpp`](./src/main/src/motor/motor.cpp)
- **Responsive ramps (per `20 ms` tick):** Forward `+18 PWM` (`0 → 255` in `0.28 s`), Turn `+22 PWM` (in-place turns seed immediately at `58 PWM` on tick 1 so zero time is wasted below stiction), Deceleration `-25 PWM`. Opposite turn requests zero `turn` immediately.
- **Turn-to-drive budget (`DRIVE_TURN_RATIO = 0.55`):** When `forward > 0`, `|turn|` is bounded to `min(0.80 * forward, max(28, 0.55 * forward))`, giving crisp capture curvature while keeping all 4 wheels strictly in their forward direction at every speed (`reversing` happens at most once on tick 1 of `Aligning → Chasing`, with zero mid-sprint `0 PWM` dropouts).
- **Shared saturation, neutral reversal & native `4,482 Hz` MDD3A output:** Uses Teensy 4.1's native `4,482.29 Hz` `8-bit` FlexPWM configuration (`analogWriteResolution(8); stop();` — well inside the `<= 20 kHz` MDD3A spec without re-programming shared FlexPWM submodules), scales `[F + T, -F + T, -F + T, F + T]` proportionally if peak `> 255`, inserts one coordinated `0 PWM` cycle if any wheel flips sign, and zeros the inactive MDD3A input *before* driving the active input.

| Channel | Physical Side | `inputA` | `inputB` | Pure Forward (`F=255, T=0`) | Pure CW Turn (`F=0, T=+120`) |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **M1** | Left Pair (`F + T`) | Pin `1` | Pin `2` | `-255` (`Pin1=0, Pin2=255`) | `-120` (`Pin1=0, Pin2=120`) |
| **M2** | Right Pair (`-F + T`) | Pin `4` | Pin `3` | `+255` (`Pin4=255, Pin3=0`) | `-120` (`Pin4=0, Pin3=120`) |
| **M3** | Right Pair (`-F + T`) | Pin `5` | Pin `6` | `-255` (`Pin5=0, Pin6=255`) | `+120` (`Pin5=120, Pin6=0`) |
| **M4** | Left Pair (`F + T`) | Pin `7` | Pin `8` | `+255` (`Pin7=255, Pin8=0`) | `+120` (`Pin7=120, Pin8=0`) |

#### 4. Scheduler, Self-Test & Non-Blocking Telemetry — [`main.ino`](./src/main/main.ino)
- Enforces a `2 s` startup countdown (`STARTING`) and a `20 ms` (`50 Hz`) control schedule; resets cleanly if stalled `> 100 ms`.
- Includes a `t` **raised-wheel motor self-test** and non-blocking `10 Hz` telemetry / `1 Hz` raw IR diagnostics (`Serial.availableForWrite()` guarded).

---

## 6. Upload, Calibration & Troubleshooting

### A. Flashing Firmware to Teensy 4.1

1. **Turn the `MOTOR` switch OFF** (USB powers Teensy + IR ring while keeping wheels unpowered).
2. Open [`src/main/main.ino`](./src/main/main.ino) in **Arduino IDE 2** (with Teensy board package `1.62.0` installed via `https://www.pjrc.com/teensy/package_teensy_index.json`).
3. Set **Tools → Board → Teensy 4.1**, **Tools → USB Type → Serial**, and select the Teensy USB port (last known: `COM9` — do not pick a Bluetooth COM port).
4. Click **Verify**, then **Upload** (press the physical Teensy program button once only if Teensy Loader prompts).
5. Open **Serial Monitor** at **`115200` baud** and confirm **`FW:IR11-PURSUIT`** is printing every `100 ms`. *(If Teensy Loader says "Press Button on Teensy to manually enter Program Mode", press the small white button on the Teensy 4.1 once so the new `.hex` actually flashes.)*

*CLI compile-only check (without launching Teensy Loader or uploading):*
```powershell
arduino-cli compile --fqbn teensy:avr:teensy41 --warnings all --build-property "recipe.hooks.postbuild.1.pattern=" ".\src\main"
```
*(Add `--build-property "compiler.cpp.extra_flags=-DSTART_PAUSED=1"` if you want a bench build that boots paused until `r` or `g` is sent.)*

### B. Serial Commands (`115200` baud, case-insensitive)

| Command | Action |
| :---: | :--- |
| **`s` / `S`** | **Immediate stop.** Halts motors or cancels active calibration, countdown, or motor test. Remains stopped until `r`, `g`, `z`, or power-cycle. |
| **`r` / `R`** or **`g` / `G`** | **Run / resume tracking.** Starts a `2 s` safety countdown using the active zero (`Cal:SAVED` or `0.0°` `Cal:DEFAULT`) without touching EEPROM. |
| **`z` / `Z`** | **Calibrate holder zero.** Stops motors, samples the IR ball for up to `3 s`, saves the fine-tuned `0°` offset to EEPROM if stable, then starts a `2 s` countdown. If calibration fails while already running (and `s` was not sent), automatically resumes with the previous valid zero. |
| **`t` / `T`** | **Raised-wheel motor self-test.** After a `2 s` warning, pulses `M1`, `M2`, `M3`, `M4` forward individually, spins **clockwise** (`0.5 s`), drives **straight forward** (`0.5 s`), and returns to `STOPPED` (any non-whitespace key aborts instantly). |
| *(any other char)* | **Ignored safely (`INFO` hint printed).** Stray serial characters never kill active autonomous tracking. |

*Important power-cycle note:* If the **`BOARD`** switch (`5Vin`) is already **ON** when you unplug USB, Teensy 4.1 does **not** reboot — so if you previously sent `s` in Serial Monitor, send **`r`** (or **`g`**) before unplugging USB, or toggle **`BOARD`** OFF → ON on the field.

### C. Optional Holder Fine-Calibration (`z`) & First Powered Check

1. **`MOTOR` switch OFF.** Keep the Teensy powered via USB and turn **ON** the pulsed IR ball.
2. *(Optional if `Cal:SAVED` is already set or `Angle` already reads `~0.0°` at the holder):* Place the uncovered ball motionless **20–30 cm dead-ahead** on the centerline of the front concave recess (`0°`), at IR-ring height. Send **`z`** and hold still until `Calibration:SAVED Zero:...` appears (`>= 40/50` frames with `Quality >= 0.30` within `±12°`, RMS spread `<= 6°`).
3. **Verify 4 cardinal angles (`MOTOR` switch still OFF):** Move the uncovered ball around the robot (both far and `< 15 cm` close) and confirm **Front ≈ `0°`**, **Right ≈ `90°`**, **Rear ≈ `180°`**, **Left ≈ `270°`**.
4. **Raised-wheel hardware check:** Prop the chassis securely so **all four wheels spin freely off the ground**. Turn **`MOTOR` switch ON** (also ensure **`BOARD` switch is ON** if unplugging USB) and optionally send **`t`** to verify `M1–M4`, `CLOCKWISE_TURN`, and `STRAIGHT_FORWARD` before sending **`r`** (or **`g`**):
   - Ball at `~45°–90°` right/left → `State:ALIGNING`, all 4 wheels spin briskly (`>= 62 PWM`) to turn the `"凹"` holder *toward* the ball (`M1–M4` LEDs on both Cytron MDD3A boards light up).
   - Ball inside `±12°` front → `State:TRACKING`, all 4 wheels drive forward at **`PWM:255/255`**, continuing in `State:PUSH_HOLD` for `2.5 s` if the ball tucks into the `"凹"` overhang blind spot.
5. Only after passing Step 4, test on a **clear, flat floor** (never a table) with both **`BOARD`** and **`MOTOR`** switches **ON**. Never leave motors stalled against a wall at full PWM.

### D. Telemetry & Fault Isolation

| Symptom / `State:` | Meaning & Exact Fix |
| :--- | :--- |
| `Cal:DEFAULT` / `Cal:FALLBACK` | Running with default `Zero:0.0` (`S1` front sensor). Works immediately; run `z` with ball `20–30 cm` ahead of recess if you want sub-degree centerline trim saved to EEPROM (`Cal:SAVED`). |
| `CALIBRATION_FAILED` | Ball moved or lacked `40/50` clear readings during `z`. Hold ball steady `20–30 cm` ahead of the `"凹"` recess (not hidden inside the recess) and resend `z`, or send `r` to run with the previous/default zero. |
| `NO_BALL` (`RawIR` all `0/0`) | No active-`LOW` signal on any pin. Verify the IR ball battery is charged, switched ON, and Teensy `3.3V` rail is present. |
| `NO_DIRECTION` (`UNIFORM_SIGNAL`) | All sensors read uniform level (e.g., `1000/0` across all 16 pins if IR `3.3V` power wire is disconnected). Inspect `RawIR` line in Serial Monitor. |
| `NO_DIRECTION` (`AMBIGUOUS_DIRECTION`) | Two equal IR sources `180°` apart. Remove second ball or mirror reflection. |
| `PUSH_HOLD` | Normal close-capture pushing (`255 PWM` for up to `2.5 s`) while the captured ball is shaded inside the `"凹"` pocket. Automatically steers if the ball slips out or stops after `2.5 s` if the ball is removed. |
| `ALIGNING` with nonzero `M1..M4`, yet chassis spins *away* from ball or translates | **Motor channel or lead-polarity mismatch** — do *not* recalibrate IR to mask this. Raise wheels and run **`t`** to check each `M1–M4` channel individually. |
| `TRACKING`, `Angle ~ 0°`, `AppliedF > 0`, `AppliedT ~ 0`, yet robot moves sideways or spins | **Drivetrain plug swap** between MDD3A channels or reversed `Motor Tab Board`. Raise wheels and run **`t`** (`STRAIGHT_FORWARD` must drive all 4 wheels forward). |
| Repeated `STARTING` mid-run | **Teensy brownout/reset** under motor current surge. Charge the 3S LiPo and check the `5.0V` LM2596 buck converter wiring on the left deck. |

---

## 7. Legacy Codebase Differences & External Links

Before importing any code from the 2025–2026 repository (`RoboCup 2027` / `Surfenn/RoboCup-2026`), note these incompatible conventions:

- **Original modular branches (`main` @ `68078e4`, `new-changes` @ `e4e5db4`):** Used sequential `pulseIn()` timeouts, an **opposite (counter-clockwise) IR angle sign**, `-90°` Bot #2 wheel-axis offsets, and `±40°` orbital ball-catching offsets.
- **BallTracker rewrite branches (`good-changes-works` @ `86bd9a6`, `New-Changes---New-Code---Working` @ `5b613ff`):** Used `PWM + DIR` MDD3A wiring tricks (`255 - pwm` duty inversion when `DIR=HIGH` and `duty=1` idle priming) plus **compass heading lock + orbital translation (`±100°` offset)** to circle behind the ball rather than turning the holder to face it. `IR11-PURSUIT` matches BallTracker's empirically verified wheel polarities using clean **`4.48 kHz` dual-PWM (`analogWrite` A/B)** commands and direct holder-facing pursuit.

### Reference Links
- **2026–2027 Onshape 3D CAD (`Germany 2026-27`):** <https://cad.onshape.com/documents/d6a9c24e179e5b10c2ce2f60/w/c426bb7fe9983fc80df2e79b/e/4f4dadfa8db7efcaffd046fc>
- **RCJ Pulsed IR Ball (Golf-Ball Size v2):** <https://rovotas.com/products/robocupjunior-soccer-ir-ball-golf-ball-size-v2>
- **Solenoid Kicker (`JF-1253B` 12V 450 mA):** <https://www.amazon.ca/JF-1253B-450mA-Stroke-Solenoid-Electromagnet/dp/B0FCX7GJ8Q>
- **Cytron MDD3A Dual Motor Driver:** <https://github.com/CytronTechnologies/CytronMotorDriver>
- **Legacy 2025–2026 Firmware Branches:** <https://github.com/Surfenn/RoboCup-2026/branches>
