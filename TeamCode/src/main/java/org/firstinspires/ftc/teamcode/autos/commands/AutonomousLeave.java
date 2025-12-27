package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.indexer.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;

import java.util.List;

public class AutonomousLeave extends SequentialCommandGroup {


    public AutonomousLeave(@NonNull DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, VisionSubsystem vision, List<Pose> poses) {

        addCommands(
                new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, poses).withTimeout(28000),
                new LeaveCommand(drivetrain, poses.get(PosesNames.GoToLine1.ordinal()))

        );
        addRequirements(drivetrain, shooter, intake);
    }

}

