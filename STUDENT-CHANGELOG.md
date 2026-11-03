# FLUX 10413 — Student Changelog (`STUDENT-CHANGELOG.md`)

## 2026-11-03 — Initial `v2027.1.0` Pre-Kickoff Baseline: Shared Core Engine, 3D Autonomous Lab & Student Exercise Kit
* **🔧 What's Included:**
  * **Shared Core Engine (`frc.lib.*`):** 250 Hz CTRE Phoenix 6 swerve drivetrain (`Drive`, `ModuleIOTalonFX`, `ModuleIOSim`), full-queue AprilTag vision (`Vision`, `VisionIOLimelightSystemCore`, `VisionIOPhotonVision`, `VisionIOSim`), generic `PositionMechanism` & `VelocityMechanism` subsystems, `VirtualGeofence` arena protection, `FaultSupervisor`, `BatteryIO` (`Playing With Fusion BFG SEN-36005` CAN monitor), and `FluxFig` CSV configuration (`libs/fluxfig-1.0.0.jar`).
  * **Student 3D Autonomous Lab (`StudentAutoRoutines.java` & `./gradlew renderAuto`):** Empty `STUDENT` starter (`createStudentDefaultAuto`) with inline cheat sheets for **Track A (Fixed Code Moves)** (`driveRelative`, `driveTimedVelocity`, `waitLoopSeconds`) and **Track B (PathPlanner `.path` Moves)** (`resetPoseToPathStart`, `followPathCommand`), rendering an interactive 3D browser viewer (`build/auto-sim/student-auto.html`) and MP4 video (`build/auto-sim/student-auto.mp4`).
  * **4-Level Progressive Student Exercise Kit (`StudentExercises.java` & `./gradlew checkExercises`):** Self-graded JUnit 5 exercises covering `FluxFig` hardware safety limits, `Superstructure` crossbar keep-out interlocks, swerve heading alignment, and 15-second autonomous routines.
* **🎯 What to Try First:**
  * Run `./gradlew renderAuto -Pauto=list` or `./gradlew renderAuto -Pauto="Fixed Code Moves"` and open `build/auto-sim/student-auto.html` in your browser!
  * Run `./gradlew checkExercises` to test your solutions in [`src/main/java/frc/robot/exercises/StudentExercises.java`](src/main/java/frc/robot/exercises/StudentExercises.java).
