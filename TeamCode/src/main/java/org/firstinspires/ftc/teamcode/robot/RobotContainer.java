package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autos.commands.AutonomousCommands;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.AimByPoseCommand;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPose;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * This class is the main container for the robot. It holds all subsystems, configures button
 * bindings, and provides commands for autonomous and tele-operated modes.
 * The RobotContainer is the "glue" that connects the robot's hardware and software components.
 */
public class RobotContainer {

    // Subsystem declarations
    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;
    /**
     * The constructor for the RobotContainer. It is responsible for initializing all
     * subsystems, setting up default commands, and configuring gamepad button bindings.
     *
     * @param hardwareMap The hardware map from the OpMode, used to initialize hardware components.
     * @param telemetry   The telemetry manager for logging and dashboard output.
     * @param driver      The driver's gamepad (GamepadEx) for controlling the robot's movement.
     * @param operator    The operator's gamepad (GamepadEx) for controlling robot mechanisms.
     */
    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator, AllianceEnum alliance) {
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap, telemetry);


        if (driver != null) {
            drivetrain.getFollower().setPose(DataStorage.actualPose);
            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));

            // Driver controller bindings
            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry, operator));
            Pose EndPose;

            double targetx, targety;
            if (alliance == AllianceEnum.Red) {
                targetx = 144;
                targety = 144;
            } else {
                targetx = 0;
                targety = 144;
            }

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whileHeld(new AimByPoseCommand(drivetrain, targetx, targety, telemetry));

            if (alliance == AllianceEnum.Red) {
                EndPose = BlueRearPoses.getPose(PosesNames.EndPose);
            } else {
                EndPose = RedRearPoses.getPose(PosesNames.EndPose);
            }

            new GamepadButton(driver, GamepadKeys.Button.A)
                    .whileHeld(new GoToPose(drivetrain,EndPose));

            Pose ShootPose;

            if (alliance == AllianceEnum.Red) {
                ShootPose = RedRearPoses.getPose(PosesNames.GoToShoot1);
            } else {
                ShootPose = BlueRearPoses.getPose(PosesNames.GoToShoot2);
            }

            new GamepadButton(driver, GamepadKeys.Button.B)
                    .whileHeld(new GoToPose(drivetrain,ShootPose));
            new GamepadButton(driver, GamepadKeys.Button.START).whenActive(new UpdatePoseLimelightCommand(drivetrain, vision, vision.getRobotPose().get()));

        }

        if (operator != null) {
            configureTeleOpBindings(operator, telemetry);
        }
    }

    public void updateRobotPose(AllianceEnum alliance, Pose robotPose) {

        double yaw = robotPose.getHeading();
        robotPose = vision.getRobotPose(yaw).orElse(robotPose);
        drivetrain.getFollower().setPose(robotPose);


        drivetrain.periodic();

        PanelsTelemetry.INSTANCE.getTelemetry().update();

    }
    /**
     * Gets the command group for the Blue Rear autonomous routine.
     *
     * @return A {@link Command} object representing the complete autonomous sequence.
     */
    public Command getAutonomousBlueRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, BlueRearPoses.asList());
    }

    /**
     * Gets the command group for the Red Rear autonomous routine.
     *
     * @return A {@link Command} object representing the complete autonomous sequence.
     */
    public Command getAutonomousRedRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, RedRearPoses.asList());
    }

    /**
     * Configures the button bindings for the operator's gamepad. This method maps specific
     * gamepad buttons to commands that control the robot's mechanisms like the intake and shooter.
     *
     * @param operator  The operator's gamepad (GamepadEx).
     * @param telemetry The telemetry manager, passed to any commands that need it.
     */
    private void configureTeleOpBindings(GamepadEx operator, TelemetryManager telemetry) {
        ShootCommand shoot = new ShootCommand(shooter, intake, indexer);

        // Hood controls
        new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(shooter::decreaseHood, shooter));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(shooter::increaseHood, shooter));

        // Continuous shooting
        new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new AutoShootCommand(drivetrain, vision, shooter, intake, indexer));
        // Stop shooter
        new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(shooter::stop, shooter));

        // Intake controls
        new GamepadButton(operator, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::runTrigger, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        // Spin shooter to preset speeds
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT));

        new GamepadButton(operator,GamepadKeys.Button.X)
                .whenPressed(new SpinShooterCommand(shooter, SpinShooterCommand.Action.LONG_SHOOT));
    }

}
