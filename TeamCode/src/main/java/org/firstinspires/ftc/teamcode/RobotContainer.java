package org.firstinspires.ftc.teamcode;

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
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autos.commands.AutonomousCommands;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.commands.drive.AimByPoseCommand;
import org.firstinspires.ftc.teamcode.commands.drive.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.drive.GoToPose;
import org.firstinspires.ftc.teamcode.commands.drive.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TeleOpDriveCommandZoneRepulsion;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
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

    private final GamepadEx driver;
    private final GamepadEx operator;

    private final AllianceEnum alliance;

    private final Pose innitialPose;
    private final Pose endPose;
    private final Pose shootPose;

    private final Command driveCommand;
    private final Command driveCommandZoneRepulsion;

    public RobotContainer(HardwareMap hardwareMap, GamepadEx driver, GamepadEx operator, AllianceEnum alliance) {

        // Subsystem Initialization
        drivetrain = DrivetrainSubsystem.getInstance(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);

        // Controller Initialization
        this.driver = driver;
        this.operator = operator;

        // Alliance Initialization
        this.alliance = alliance;

        innitialPose = (alliance == AllianceEnum.Red)?
                RedRearPoses.getPose(PosesNames.EndPose) : BlueRearPoses.getPose(PosesNames.EndPose);
        //TODO: Lucas, vê se esse initial pose deveria
        // ser com esse ".getPose(PosesNames.EndPose)" porque me parece estranho um "end" no "innitial"

        endPose = (alliance == AllianceEnum.Red)?
                BlueRearPoses.getPose(PosesNames.EndPose) : RedRearPoses.getPose(PosesNames.EndPose);
        shootPose = (alliance == AllianceEnum.Red)?
                RedRearPoses.getPose(PosesNames.GoToShoot1) : BlueRearPoses.getPose(PosesNames.GoToShoot1);

        // Normal DriveCommand Initialization
        driveCommand = new TeleOpDriveCommand(drivetrain, driver);

        // Repulsion DriveCommand Initialization
        Polygon2d triangleBig = new Polygon2d(new Translation2d(72, 72), new Translation2d(144, 144), new Translation2d(0, 144));
        driveCommandZoneRepulsion = new TeleOpDriveCommandZoneRepulsion(
                drivetrain,
                driver,
                triangleBig,
                1.0
        );

        Command periodicUpdateLoop = new UpdatePoseLimelightCommand(drivetrain, vision, () -> !isRobotShooting());

        CommandScheduler.getInstance().schedule(periodicUpdateLoop);

        if (driver!= null) {
            configureTeleOpBindingsDriver();
        }

        if (operator!= null) {
            configureTeleOpBindingsOperator();
        }
    }

    private void configureTeleOpBindingsDriver() {
        if (DataStorage.actualPose != null) {
            drivetrain.getFollower().setPose(DataStorage.actualPose);
        } else {
            Pose startPose = (alliance == AllianceEnum.Red)?
                    RedRearPoses.getPose(PosesNames.EndPose) : BlueRearPoses.getPose(PosesNames.EndPose);
            drivetrain.getFollower().setPose(startPose);
        }

        drivetrain.setDefaultCommand(driveCommand);

        new GamepadButton(driver, GamepadKeys.Button.Y)
                .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, operator));

        new GamepadButton(driver, GamepadKeys.Button.BACK)
                .whenPressed(new UpdatePoseLimelightCommand(drivetrain, vision, () -> true));

        new GamepadButton(driver, GamepadKeys.Button.A)
                .whileHeld(new GoToPose(drivetrain, endPose));

        new GamepadButton(driver, GamepadKeys.Button.B)
                .whileHeld(new GoToPose(drivetrain, shootPose));

        double targetx = (alliance == AllianceEnum.Red)? 144 : 0;
        double targety = 144;

        new GamepadButton(driver, GamepadKeys.Button.X)
                .whileHeld(new AimByPoseCommand(drivetrain, targetx, targety));
    }

    private void configureTeleOpBindingsOperator() {
        new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new AutoShootCommand(drivetrain, vision, shooter, intake, indexer, endPose));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(shooter::decreaseHood, shooter));
        new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(shooter::increaseHood, shooter));

        new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooter::stop, shooter));

        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::runTrigger, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT));
        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.LONG_SHOOT));
    }

    public void updateRobotPose(Pose robotPose) {
        robotPose = vision.getRobotPose().orElse(robotPose);
        drivetrain.getFollower().setPose(robotPose);
        drivetrain.periodic();
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public Command getAutonomousBlueRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, BlueRearPoses.asList());
    }

    public Command getAutonomousRedRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, RedRearPoses.asList());
    }
    private boolean isRobotShooting() {
        Command current = drivetrain.getCurrentCommand();
        return current instanceof AutoShootCommand ||
                current instanceof AimByPoseCommand ||
                current instanceof AlignToAprilTagCommand;
    }
}