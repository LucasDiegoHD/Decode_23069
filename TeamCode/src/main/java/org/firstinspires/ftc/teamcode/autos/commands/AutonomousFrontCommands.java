package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.List;

public class AutonomousFrontCommands extends SequentialCommandGroup {

    public AutonomousFrontCommands(@NonNull DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, VisionSubsystem vision, List<Pose> poses, LEDSubsystem ledSubsystem) {

        addCommands(
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.StartPose.ordinal())),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())).withTimeout(1500),
                new WaitUntilCommand(shooter::isReady).withTimeout(800),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine2.ordinal())
                ).withTimeout(4000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot2.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitCommand(800),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine1.ordinal()),
                        poses.get(PosesNames.CatchLine1.ordinal())
                ).withTimeout(4000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot2.ordinal())).withTimeout(2000),
                new WaitCommand(800),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GatePose.ordinal())).withTimeout(2000),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.CatchLine3.ordinal())).withTimeout(2000),
                new WaitCommand(600),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())).withTimeout(2000),
                new WaitCommand(800),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.EndPose.ordinal())).withTimeout(2000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.STOP)
        );
        addRequirements(drivetrain, shooter, intake);
    }
}