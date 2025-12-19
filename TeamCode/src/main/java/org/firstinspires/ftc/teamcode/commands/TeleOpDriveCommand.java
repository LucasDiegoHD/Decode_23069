package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * A command for controlling the robot's drivetrain during the tele-operated period.
 * It uses field-centric control, meaning the robot's movement is relative to the field,
 * not the robot's orientation.
 */
public class TeleOpDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;
    private final AllianceEnum alliance;

    /**
     * Creates a new TeleOpDriveCommand.
     *
     * @param drivetrain    The DrivetrainSubsystem to control.
     * @param driverGamepad The gamepad used for driving.
     */
    public TeleOpDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.alliance = DataStorage.alliance;
        addRequirements(drivetrain);
    }

    /**
     * Called when the command is initially scheduled. Prepares the follower for teleop mode.
     */
    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
    }

    /**
     * Called repeatedly while the command is scheduled. Reads joystick inputs,
     * calculates field-centric drive vectors, and commands the drivetrain.
     */
    @Override
    public void execute() {
        Pose p = drivetrain.getFollower().getPose();
        double heading = p.getHeading();
        // Read joystick values
        double y = driverGamepad.getLeftX(); // Forward/backward
        double x = driverGamepad.getLeftY(); // Strafe left/right

        // Field-centric transformation
        double xField = x * Math.cos(heading) - y * Math.sin(heading);
        double yField = x * Math.sin(heading) + y * Math.cos(heading);
        if(alliance == AllianceEnum.Blue){
            xField =-xField;
            yField =-yField;
        }
        // Apply power to the drivetrain
        drivetrain.getFollower().setTeleOpDrive(
                -yField, // Forward/backward power
                -xField, // Strafe power
                -driverGamepad.getRightX(), // Turn power
                true // Specify that the control is field-centric
        );
        PanelsTelemetry.INSTANCE.getTelemetry().addData("Alliance", alliance);

    }

}