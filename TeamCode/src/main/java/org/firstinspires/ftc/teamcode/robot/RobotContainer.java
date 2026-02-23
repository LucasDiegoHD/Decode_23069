package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.autos.commands.AutonomousCommands;
import org.firstinspires.ftc.teamcode.autos.commands.AutonomousFrontCommands;
import org.firstinspires.ftc.teamcode.autos.commands.AutonomousTuffCommand;
import org.firstinspires.ftc.teamcode.autos.paths.BlueFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.Polygon2d;

/**
 * Main container for robot organization.
 * Features Auto-Periodic Limelight Resync and high-level drive scaling.
 */
public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;
    private final LEDSubsystem led;
    private final HuskySubsystem husky;
    private boolean isShooterAutoAdjustActive = true;


    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator, AllianceEnum alliance) {
        // Subsystem Initialization
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap);
        husky = new HuskySubsystem(hardwareMap,telemetry);

        led.setDefaultCommand(new LedCommand(led, indexer));


        Polygon2d triangleBig = new Polygon2d(new Translation2d(72, 72), new Translation2d(144, 144), new Translation2d(0, 144));

        Command driveCommandZoneRepulsion = new TeleOpDriveCommandZoneRepulsion(
                drivetrain,
                driver,
                triangleBig,
                1.0
        );

        if (driver!= null) {
            if (DataStorage.actualPose!= null) {
                drivetrain.getFollower().setPose(DataStorage.actualPose);
            } else {
                Pose startPose = (alliance == AllianceEnum.Red)?
                        RedRearPoses.getPose(PosesNames.StartPose) : BlueRearPoses.getPose(PosesNames.StartPose);
                drivetrain.getFollower().setPose(startPose);
            }


            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));

            Pose endPose = (alliance == AllianceEnum.Red)?
                    BlueRearPoses.getPose(PosesNames.EndPose) : RedRearPoses.getPose(PosesNames.EndPose);
            Pose innitialPose = (alliance == AllianceEnum.Red)?
                    RedRearPoses.getPose(PosesNames.EndPose) : BlueRearPoses.getPose(PosesNames.EndPose);
            Pose shootPose = (alliance == AllianceEnum.Red)?
                    RedRearPoses.getPose(PosesNames.GoToShoot1) : BlueRearPoses.getPose(PosesNames.GoToShoot1);

            double goalX = (alliance == AllianceEnum.Red)? 130 : 14;
            double goalY = 130;

            Command periodicUpdateLoop = new RepeatCommand(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new ConditionalCommand(
                                    new InstantCommand(),
                                    new UpdatePoseLimelightCommand(drivetrain, vision, innitialPose),
                                    () -> isRobotShooting() || !drivetrain.isRobotStopped()
                            )
                    )
            );

            CommandScheduler.getInstance().schedule(periodicUpdateLoop);

            Command AdjustHood = new RepeatCommand(
                    new SequentialCommandGroup(
                            new WaitCommand(200),
                            new AdjustHoodCommand(shooter, vision, drivetrain, goalX, goalY)
                    )
            );

            CommandScheduler.getInstance().schedule(AdjustHood);



            Command AdjustShooter = new RepeatCommand(
                    new SequentialCommandGroup(
                            new WaitCommand(200),
                            new ConditionalCommand(
                                    new AdjustShooterCommand(shooter, vision, drivetrain, goalX, goalY),
                                    new InstantCommand(),
                                    () -> isShooterAutoAdjustActive
                            )
                    )
            );

            CommandScheduler.getInstance().schedule(AdjustShooter);


            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry, operator));

            new GamepadButton(driver, GamepadKeys.Button.A)
                    .whileHeld(new GoToPose(drivetrain, endPose));

            new GamepadButton(driver, GamepadKeys.Button.B)
                    .whileHeld(new GoToPose(drivetrain, shootPose));

            double targetx = (alliance == AllianceEnum.Red)? 144 : 0;
            double targety = 144;

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whileHeld(new AimByPoseCommand(drivetrain, targetx, targety, telemetry));

            new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT)
                    .whileHeld(new ChaseArtifactCommand(drivetrain,husky,intake));

            new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new InstantCommand(() -> {
                        isShooterAutoAdjustActive = false;
                        shooter.stop();
                    }));

            new GamepadButton(driver, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(new InstantCommand(() -> isShooterAutoAdjustActive = true));

            new GamepadButton(driver, GamepadKeys.Button.START)
                    .whenPressed(new InstantCommand(() -> {
                        double targetAngle = 90.0;

                        UpdatePoseLimelightCommand.forceHardReset(drivetrain, vision, targetAngle);
                    }));

            new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new InstantCommand(() -> {
                        double targetAngle = 40.0;

                        UpdatePoseLimelightCommand.forceHardReset(drivetrain, vision, targetAngle);
                    }));

            new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(new InstantCommand(() -> {
                        double targetAngle = 130.0;

                        UpdatePoseLimelightCommand.forceHardReset(drivetrain, vision, targetAngle);
                    }));
        }

        if (operator!= null) {
            configureTeleOpBindings(operator, alliance);
        }
    }

    private void configureTeleOpBindings(GamepadEx operator, AllianceEnum alliance) {
        Pose endPose = (alliance == AllianceEnum.Red)?
                BlueRearPoses.getPose(PosesNames.EndPose) : RedRearPoses.getPose(PosesNames.EndPose);

        new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new AutoShootCommand(drivetrain, vision, shooter, intake, indexer, endPose, led));

        new GamepadButton(operator, GamepadKeys.Button. LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(intake::runTrigger, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    isShooterAutoAdjustActive = false;
                    shooter.stop();
                }));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> isShooterAutoAdjustActive = true));

    }

    public void updateRobotPose(AllianceEnum alliance, Pose robotPose) {
        double yaw = robotPose.getHeading();
        robotPose = vision.getRobotPose(yaw).orElse(robotPose);
        drivetrain.getFollower().setPose(robotPose);
        drivetrain.periodic();
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public Command getAutonomousBlueRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, BlueRearPoses.asList(), led);
    }

    public Command getAutonomousRedRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, RedRearPoses.asList(), led);
    }

    public Command getAutonomousBlueFrontCommand() {
        return new AutonomousFrontCommands(drivetrain, shooter, intake, indexer, vision, BlueFrontPoses.asList(), led);
    }

    public Command getAutonomousRedFrontCommand() {
        return new AutonomousFrontCommands(drivetrain, shooter, intake, indexer, vision, RedFrontPoses.asList(), led);
    }

    public Command getAutonomousRedTuffCommand() {
        return new AutonomousTuffCommand(drivetrain, shooter, intake, indexer, vision, RedRearPoses.asList(), led);
    }

    public Command getAutonomousBlueTuffCommand() {
        return new AutonomousTuffCommand(drivetrain, shooter, intake, indexer, vision, BlueRearPoses.asList(), led);
    }
    private boolean isRobotShooting() {
        Command current = drivetrain.getCurrentCommand();
        return current instanceof AlignToAprilTagCommand;
    }
}