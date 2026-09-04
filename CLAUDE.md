# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FTC robot code for team **23069** (Brazilian team, DECODE season). Built on the FTC SDK
(`v11.2.1`), **Pedro Pathing** for motion, and its **Ivy** command-based framework
(`com.pedropathing:ivy:1.0.0`). FTCLib was removed — see `MIGRACAO-IVY.md`.
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

**Entry points** (`robot/RobotOpMode` subclasses): `teleop.java` and `autos/Autos.java`
(a gamepad-driven selector for alliance + strategy). `RobotOpMode extends OpMode` — iterative,
not `LinearOpMode`: it resets Ivy's static `Scheduler` in `init()` and `stop()`, builds the
`Robot`, schedules one `Commands.infinite(robot::update)`, and calls `Scheduler.execute()` once
per `init_loop`/`loop`.

**`robot/Robot`** is the wiring hub: constructs the six subsystems and owns the single continuous
loop. `Robot.update()` clears the Lynx bulk cache, then calls each subsystem's `update()` in a
fixed, explicit order. Gamepad bindings and autonomous routine assembly live in the OpModes, not
here.

**Commands** are `static` factories returning `com.pedropathing.ivy.Command`, built with
`Command.build()` / `Commands.*` / `Groups.*`. There is no default-command concept in Ivy: the
continuous drive and aim commands run at priority 0 with `InterruptedBehavior.SUSPEND`, and
button commands reserve the same subsystem at priority 1 — the scheduler suspends and later
resumes the continuous one on its own. Bindings are polled in `loop()` with the SDK's
`gamepadX.<button>WasPressed()` helpers; there is no `GamepadEx`.

**Subsystems** — each is a plain class (no framework base) exposing `void update()`, paired with
a `*Constants` class (many `@Configurable`
for live tuning via the Panels dashboard):
- `subsystems/`: `DrivetrainSubsystem` (wraps Pedro `Follower`), `IntakeSubsystem`, `VisionSubsystem`.
- `subsystems/templates/`: `ShooterSubsystem`, `IndexerSubsystem`, `LEDSubsystem`. All six are in
  active use; the unused Climber/Elevator/Husky templates were deleted during the migration.

**Drivetrain / pathing**: mecanum + GoBilda Pinpoint localizer. All Pedro tuning lives in
`pedroPathing/Constants.java` (`createFollower(hardwareMap)` is the single construction point).

**Vision**: Limelight 3A. AprilTag pose estimates are fused into Pinpoint odometry via
`commands/UpdatePoseLimelightCommand` (`forceHardReset` on driver START). Pipeline file: `limelight/apriltags.vpr`.

**Shooter**: closed-loop velocity PIDF with battery-voltage compensation, distance→RPM polynomial
(Horner form), dynamic dual hood servos, and a live RPM offset the operator trims with the dpad.

**Autonomous paths**: `autos/paths/{Red,Blue}{Front,Rear}Poses.java` each hold a `Pose[] POSES`
indexed by the **`PosesNames` enum ordinal** — the array order must stay in lockstep with the enum.
`autos/AutoRoutines.java` holds the three routines as `static` factories (`rearNormal`, `front`,
`rearNoGate`) built from `Groups.sequential` / `Groups.parallel`, each consuming a pose list.
`autos/commands/GoToPoseCommand` is a fluent builder ending in `toCommand()`.

**Cross-OpMode state**:
- `utils/DataStorage` — static in-memory: `alliance`, `actualPose`, `pieceCount`, `DEBUG_MODE`.
- `utils/PoseStorage` — persists pose to `/sdcard/stored_robot_pose.txt` so teleop resumes where
  auto ended. `Robot.applyTeleOpStartPose` prefers `DataStorage.actualPose`, then the file, then
  a start pose.
- Alliance is chosen in the Auto selector and read back in teleop.

**Loop hygiene**: Lynx hubs run `BulkCachingMode.MANUAL`. The bulk cache is cleared exactly once
per iteration, as the first line of `Robot.update()` — do not clear it in the OpMode too, or every
cycle pays for two invalidations.

## Known state

The FTCLib-to-Ivy migration is complete and the build is green: `:TeamCode:compileDebugJavaWithJavac`
reports zero errors and `assembleDebug` produces the APK. No `com.arcrobotics.ftclib` import
remains. What is still unverified is on-robot behaviour — the migration's acceptance criterion is
parity with the previous behaviour, and tasks 8.3-8.5 in
`openspec/changes/migrate-ftclib-to-ivy/tasks.md` list the bench checks.

Building requires a `local.properties` with `sdk.dir` (gitignored) and a JDK 17-21. The Android
Studio `jbr` (JDK 21) that `JAVA_HOME` points to works; the `jdk-25` on `PATH` does not, since
Gradle 8.9 does not support it. Note that `javac` truncates at 100 errors by default, so measuring
a real error count needs `-Xmaxerrs` raised via an init script.
