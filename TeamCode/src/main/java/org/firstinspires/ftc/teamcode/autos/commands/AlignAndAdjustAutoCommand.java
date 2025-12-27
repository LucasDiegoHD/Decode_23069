package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.AdjustHoodCommand;
import org.firstinspires.ftc.teamcode.commands.AdjustShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

public class AlignAndAdjustAutoCommand extends SequentialCommandGroup {
    public AlignAndAdjustAutoCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter) {
        addCommands(

                //new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(500),

                new AdjustShooterCommand(shooter, vision),
                new AdjustHoodCommand(shooter, vision)

        );
        addRequirements(drivetrain, shooter);
    }
}
