package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AdjustAutoCommand extends SequentialCommandGroup {
    public AdjustAutoCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter) {
        addCommands(
               //new AdjustShooterCommandAuto(shooter, vision),
                //new AdjustHoodCommandAuto(shooter, vision),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(400)


        );
        addRequirements(shooter);
    }
}
