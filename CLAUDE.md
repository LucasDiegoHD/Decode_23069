# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FTC robot code for team **23069** (Brazilian team, DECODE season). Built on the FTC SDK
(`v11.2.1`), the **FTCLib** command-based framework, and **Pedro Pathing** for motion.
Comments, telemetry strings, and some identifiers are in Portuguese — this is normal, match
the surrounding language when editing a file.

## Build & deploy

Windows shell here is PowerShell; use `gradlew.bat` (or `./gradlew` from Git Bash).

- `./gradlew :TeamCode:compileDebugJavaWithJavac` — fast compile check, use this to verify changes
- `./gradlew :TeamCode:assembleDebug` — build the Robot Controller APK
- `./gradlew installDebug` — deploy to a connected Robot Controller (adb), or use Android Studio Run
- `./gradlew generateDebugJavadoc` — regenerates the checked-in `docs/` Javadoc site

No unit tests exist. There is no lint config beyond Android defaults. "Tests" in FTC are
Tuning/Test OpModes run on the robot (`pedroPathing/Tuning.java`, `templates/ElevatorTestOpMode.java`).

## Module layout

- `FtcRobotController/` — stock FTC SDK. Do not edit.
- `TeamCode/` — all robot code, package `org.firstinspires.ftc.teamcode`.
- OpModes are auto-registered by `@TeleOp` / `@Autonomous` annotations (annotation processor in
  `TeamCode/lib/OpModeAnnotationProcessor.jar`). There is no manual registration list.

## Architecture

**Entry points** (`CommandOpMode` subclasses): `teleop.java`, `autos/Autos.java` (a gamepad-driven
selector for alliance + strategy), and the individual `autos/Auto{Red,Blue}{Front,Rear,Tuff}.java`.
Every OpMode builds a single `robot/RobotContainer`.

**`RobotContainer`** is the wiring hub: constructs all subsystems, sets default commands, binds
gamepad buttons, and exposes `getAutonomous{Red,Blue}{Rear,Front,Tuff}Command()`. Auto passes
`null` gamepads; teleop passes real ones. It also runs a periodic Limelight re-localization loop
and toggles shooter auto-aim.

**Subsystems** — each is a `SubsystemBase` paired with a `*Constants` class (many `@Configurable`
for live tuning via the Panels dashboard):
- `subsystems/`: `DrivetrainSubsystem` (wraps Pedro `Follower`), `IntakeSubsystem`, `VisionSubsystem`.
- `subsystems/templates/`: `ShooterSubsystem`, `IndexerSubsystem`, `LEDSubsystem` are in active use.
  `ClimberSubsystem`, `ElevatorSubsystem`, `HuskySubsystem` are unused templates.

**Drivetrain / pathing**: mecanum + GoBilda Pinpoint localizer. All Pedro tuning lives in
`pedroPathing/Constants.java` (`createFollower(hardwareMap)` is the single construction point).

**Vision**: Limelight 3A. AprilTag pose estimates are fused into Pinpoint odometry via
`commands/UpdatePoseLimelightCommand` (`forceHardReset` on driver START). Pipeline file: `limelight/apriltags.vpr`.

**Shooter**: closed-loop velocity PIDF with battery-voltage compensation, distance→RPM polynomial
(Horner form), dynamic dual hood servos, and a live RPM offset the operator trims with the dpad.

**Autonomous paths**: `autos/paths/{Red,Blue}{Front,Rear}Poses.java` each hold a `Pose[] POSES`
indexed by the **`PosesNames` enum ordinal** — the array order must stay in lockstep with the enum.
`autos/commands/Autonomous*Commands.java` are `SequentialCommandGroup`s that consume a pose list.

**Cross-OpMode state**:
- `utils/DataStorage` — static in-memory: `alliance`, `actualPose`, `pieceCount`, `DEBUG_MODE`.
- `utils/PoseStorage` — persists pose to `/sdcard/stored_robot_pose.txt` so teleop resumes where
  auto ended. `RobotContainer` prefers `DataStorage.actualPose`, then the file, then a start pose.
- Alliance is chosen in the Auto selector and read back in teleop.

**Loop hygiene**: Lynx hubs run `BulkCachingMode.MANUAL`; each OpMode's `run()` calls
`robot.clearBulkCache()` once per loop before `super.run()`.

## Known state

`master` may not compile as-is: `RobotContainer.java` still does `import
org.firstinspires.ftc.teamcode.subsystems.*` and references `ShooterSubsystem`,
`IndexerSubsystem`, `LEDSubsystem` unqualified, but commit `5be8d9b` moved those classes to
`subsystems.templates`. Fix the imports in `RobotContainer` (and check `teleop.java` / `Autos.java`)
before building. Run the `compileDebugJavaWithJavac` check first.
