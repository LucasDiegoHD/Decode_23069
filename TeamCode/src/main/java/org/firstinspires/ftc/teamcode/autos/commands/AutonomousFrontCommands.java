package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.List;

public class AutonomousFrontCommands extends SequentialCommandGroup {

    public AutonomousFrontCommands(@NonNull DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, VisionSubsystem vision, List<Pose> poses, LEDSubsystem ledSubsystem) {

        addCommands(
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.StartPose.ordinal())),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommand(shooter, intake, indexer,3, ledSubsystem ).withTimeout(5000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine1.ordinal()),
                        poses.get(PosesNames.CatchLine1.ordinal()),
                        poses.get(PosesNames.GatePose.ordinal())
                ).withTimeout(4000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommand(shooter, intake, indexer,3, ledSubsystem).withTimeout(5000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine2.ordinal())
                ).withTimeout(4000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())).withTimeout(2000),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommand(shooter, intake, indexer,3, ledSubsystem).withTimeout(5000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.SHORT_SHOOT),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine3.ordinal()),
                        poses.get(PosesNames.CatchLine3.ordinal())
                ).withTimeout(4000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())).withTimeout(2000),
                new WaitCommand(500),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommand(shooter, intake, indexer,3, ledSubsystem).withTimeout(5000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.EndPose.ordinal())).withTimeout(2000),
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.STOP)
        );
        addRequirements(drivetrain, shooter, intake);
    }
}