# AGENTS.md — Flux-SystemCore-2027 Repository Guide (FRC Team 10413)

> **Standalone Repository Guide:** This `AGENTS.md` governs only `Flux-SystemCore-2027` and is 100% self-contained for single-repository clones.

---

## 1. Repository Overview & Key Files
`Flux-SystemCore-2027` is the **2027 SystemCore (`SYSTEMCORE_2027`, with 1-cell `ROBORIO_CANIVORE_2026` fallback) robot codebase** for **FRC Team 10413 (FLUX Robotics)**:
- [`README.md`](README.md) & [`STUDENTS.md`](STUDENTS.md): Repository overview, auto-generated class/dependency map ([`docs/class-map.html`](docs/class-map.html)), `FluxFig` mechanism table, and active `TODO(student)` board.
- [`STUDENT-CHANGELOG.md`](STUDENT-CHANGELOG.md): Student-readable digest of every code and CSV change.
- [`src/main/java/frc/robot/exercises/StudentExercises.java`](src/main/java/frc/robot/exercises/StudentExercises.java) & [`src/main/java/frc/robot/autos/StudentAutoRoutines.java`](src/main/java/frc/robot/autos/StudentAutoRoutines.java): 4-level student exercise kit (`./gradlew checkExercises`) and autonomous sandbox (`./gradlew renderAuto`).
- [`src/main/deploy/configs/`](src/main/deploy/configs/mechanisms.csv): Runtime `FluxFig` CSVs ([`parameters.csv`](src/main/deploy/configs/parameters.csv), [`mechanisms.csv`](src/main/deploy/configs/mechanisms.csv), [`swerve_modules.csv`](src/main/deploy/configs/swerve_modules.csv), [`cameras.csv`](src/main/deploy/configs/cameras.csv), [`batteries.csv`](src/main/deploy/configs/batteries.csv), [`releases.csv`](src/main/deploy/configs/releases.csv)) loaded by [`libs/fluxfig-1.0.0.jar`](libs/fluxfig-1.0.0.jar).

### Essential Commands (Run from Repository Root)
```bash
./gradlew updateStudentDocs spotlessApply test build   # Format, sync STUDENTS.md, and run all Tier 1-5 JUnit tests
./gradlew checkExercises                               # Grade 4-level StudentExercises.java
./gradlew renderAuto                                   # Simulate StudentAutoRoutines.java (15s) -> build/auto-sim/student-auto.{html,mp4}
./gradlew simulateJava                                 # Launch interactive WPILib Sim (AdvantageScope nt4://localhost)
./gradlew deploy                                       # Fast ~6s spot-deploy to robot over Wi-Fi/USB
```

---

## 2. Target Robots & SystemCore Hardware Rules (`RobotIdentity`)
Robots identify at boot via `RobotController.getSerialNumber()` (with fallback to `RobotController.getComments()`):

| Robot ID (`RobotIdentity`) | Controller Profile | CAN Bus Mapping | Vision & Superstructure |
| :--- | :--- | :--- | :--- |
| **`COMP_2027`** / **`PRACTICE_2027`** | `SYSTEMCORE_2027` *(or `ROBORIO_CANIVORE_2026` in `parameters.csv`)* | • **SystemCore:** `CAN 0` (`can_s0` swerve), `CAN 2` (`can_s2` mech), `CAN 4` (`can_s4` aux)<br>• **Fallback:** `"Drivetrain"` + `"Mech"` CANivores + `"rio"` aux | • **SystemCore:** 4x OV9281 on root `USB 0..3` (`limelightsc0..3`)<br>• **Superstructure:** `Elevator`, `ArmPivot`, `IntakeRoller` (`ElevatorArmKinematics`) |

### Hardware Constraints (NEVER Violate)
1. **Even-Numbered SystemCore CAN Ports Only (`can_s0`, `can_s2`, `can_s4`):** `(CAN 0 + CAN 1)` and `(CAN 3 + CAN 4)` share internal SPI buses (`14.3k` unshared / `17.7k` shared FPS ceiling on OS >= Rel 17, `SystemcoreTesting` Issue #342), and all `can_s*` ports run at **1 Mbps CAN 2.0** (no CAN FD in OS <= Beta 18, `#412`/`#417`). Call `ParentDevice.optimizeBusUtilizationForAll(...)` and leave `CAN 1` (`can_s1`) and `CAN 3` (`can_s3`) empty.
2. **Inline Battery Fuel Gauge (`BFG` `ROB-70001`) on `auxCanBus`:** Wire the 2-pin `BFG` CAN lead strictly to `auxCanBus` (`"can_s4"` at 1 Mbps CAN 2.0 on SystemCore, or `"rio"` on roboRIO fallback—never on a CAN FD bus). `BatteryIO.java` decodes little-endian 29-bit frames for Mfg ID `11` (`0x0B`), APIs `0x000`, `0x004`, and `0x01C` (not `SEN-36005` ToF or `0x010–0x012`).
3. **Direct Root USB Cameras (`USB 0`–`USB 3`):** Plug 4x OV9281 cameras directly into root ports `USB 0..3` (`Limelight.SYSTEMCORE_USB0..3`). Never use a USB hub or call `CameraServer.startAutomaticCapture()`.
4. **OS-Agnostic Log Paths (`LogFileManager`):** Never hardcode `/home/lvuser` (SystemCore uses `/home/systemcore`). Resolve `/U/logs` first, then `new File(Filesystem.getOperatingDirectory(), "logs")`, and keep `SignalLogger.enableAutoLogging(false)` in `Robot.java`.

---

## 3. Package Boundaries (`frc.lib.*` vs. `frc.robot.*`) & Loop Standards
1. **`frc.lib.*` (`41` Core Engine Classes) vs. `frc.robot.*` (`8` 2027 Robot Classes):**
   - `frc.lib.*` (`drive`, `vision`, `mechanism`, `safety`, `commands`, `util`) must **never** import `frc.robot.*` (enforced by `Tier1ArchitectureHygieneTest.testFrcLibNeverImportsFrcRobotMechanisms`).
   - Students edit `frc.robot.*` (`RobotContainer.java`, `Superstructure.java`, `ElevatorArmKinematics.java`, `StudentAutoRoutines.java`, `StudentExercises.java`) and `src/main/deploy/configs/mechanisms.csv`.
2. **Pure AdvantageKit IO & Zero `SmartDashboard` / `Preferences`:**
   - Subsystems interact with hardware strictly via `*IO` interfaces (`ModuleIO`, `GyroIO`, `VisionIO`, `MechanismIO`, `BatteryIO`) with matching `*IOSim` implementations.
   - `SmartDashboard`, `SendableChooser`, `Preferences`, and `java.awt.*` in `src/main/java/` are banned by Tier 1 CI. Use `Logger.recordOutput("Subsystem/Key", value)` and `LoggedTunableNumber`.
3. **Zero Per-Loop Allocations, Google Java Style & Batched CAN Signals:**
   - Follow the **Google Java Style Guide** (`camelCase` instance/static fields with zero `m_` or `s_` prefixes, `UPPER_SNAKE_CASE` constants, and `google-java-format` via `./gradlew spotlessApply`).
   - Reuse `private final` CTRE `ControlRequest` fields (`VelocityVoltage`, `MotionMagicExpoVoltage`, `PositionVoltage`, `VoltageOut`). Never call blocking `TalonFX.getConfigurator().apply(...)` inside the 50 Hz loop; scale brownout demand via `brownoutDemandScale` in `Drive.runVelocity()`.
   - Batch 50 Hz signals via `PhoenixSignals.register(...)` and 250 Hz swerve odometry via `PhoenixOdometryThread`.
4. **Vision & Field Coordinates (`Vision.java`, `FieldPositions.java`, `PreMatchCalibrator.java`):**
   - Drain all unread frames from every camera every cycle. Seed Pigeon 2 yaw via `PreMatchCalibrator` (MegaTag1 while Disabled) and switch to MegaTag2 when Enabled.
   - Always use `FieldPositions.isInsideField(x, y, margin)`, `FieldPositions.forAlliance(...)`, and `FieldPositions.operatorForward()` (`CENTER_RED_2027` vs. `WALL_BLUE_2026`; `FULL_FIELD`, `HALF_FIELD_BLUE/RED`, `SHOP_5X3`; NWU $+X$ forward, $+Y$ left, $+CCW$).
5. **Motor Safety & `Superstructure` Interlocks:**
   - Every `TalonFXConfiguration` must enable `StatorCurrentLimit` (`60 A` drive), `SupplyCurrentLimit` (`40 A` drive), and `SlipCurrent <= 60 A` (since `withSlipCurrent` overwrites `StatorCurrentLimit`).
   - Route multi-mechanism transitions (`Elevator`, `ArmPivot`, `IntakeRoller`) through `Superstructure.java` (`STOWED`, `INTAKING`, `SCORING_LOW`, `SCORING_HIGH`, `EJECTING`) guarded by `ElevatorArmKinematics`.

---

## 4. Verification Gate, Repo Gotchas & Student Coaching
- **Same-Commit Docs Rule:** Whenever any Java class, CSV config, or `TODO(student)` changes, run `./gradlew updateStudentDocs spotlessApply test` and append a 3-line entry (*🔧 What Changed*, *🎮 How It Affects Sim/Robot*, *🎯 What to Try Next*) to [`STUDENT-CHANGELOG.md`](STUDENT-CHANGELOG.md) in the same commit.
- **Simulation & Test Gotchas:**
  - In `MechanismIOSim` (`ARM_COSINE`), keep `armRadPerPosRot = (spanRot > 2.0) ? (2.0 * Math.PI / armGearing) : (2.0 * Math.PI)` to distinguish direct-angle joints from geared lead-screws.
  - In `Tier3CtreSimHardwareTest`, keep class-scoped `keepAliveTalon` on `"rio"` and call `DriverStationSim.setEnabled(true)` + `Unmanaged.feedEnable(100)` inside 250 Hz sim loops.
  - In JUnit tests and `StudentAutoRoutines`, use `withLoopTimeout(Command, double)` rather than `SimHooks.stepTiming` so `Timer.getFPGATimestamp()` is never frozen across tests.
  - Commit SVG badges locally in [`docs/badges/`](docs/badges/tier1-hygiene.svg) using relative links in `README.md`.
- **Agent Student Coaching Mode:** When scaffolding student tasks, provide clear Javadocs, `// TODO(student)` steps, plain-English physics/coordinate explanations, and a pre-written JUnit 5 test.
