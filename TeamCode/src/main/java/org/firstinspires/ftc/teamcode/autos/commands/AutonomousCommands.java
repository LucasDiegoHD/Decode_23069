package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import java.util.List;

public class AutonomousCommands extends SequentialCommandGroup {

    public AutonomousCommands(@NonNull DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, VisionSubsystem vision, List<Pose> poses, LEDSubsystem ledSubsystem) {

        addCommands(
                // === INÍCIO ===
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.StartPose.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),

                // === TIRO 1 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitUntilCommand(shooter::getShooterAtTarget).withTimeout(1700),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(300),
                new ShootCommandAutonomous(shooter, intake, indexer, 3).withTimeout(3000),

                // === BUSCA LINHA 1 ===
                new ParallelCommandGroup(
                        new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine1.ordinal()),
                        poses.get(PosesNames.CatchLine1.ordinal())
                ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withTimeout(4000),
                        new RunCommand(() -> {
                            if (!indexer.isFull()) intake.run();
                            else intake.stop();
                        }
                )),

                // === TIRO 2 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(400),
                new ShootCommandAutonomous(shooter, intake, indexer, indexer::getPieceCount).withTimeout(3000),

                // === BUSCA LINHA 3 ===
                new ParallelCommandGroup(
                        new GoToPoseCommand(drivetrain, true,
                                poses.get(PosesNames.GoToLine3.ordinal()),
                                poses.get(PosesNames.CatchLine3.ordinal())
                        ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withTimeout(4000),
                        new RunCommand(() -> {
                            if (!indexer.isFull()) intake.run();
                            else intake.stop();
                        }
                )),

                // === GATE ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GatePose.ordinal()))
                        .setConstraints(Constants.autoShootConstraints).withTimeout(1000),

                // === TIRO 3 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(400),
                new ShootCommandAutonomous(shooter, intake, indexer, indexer::getPieceCount).withTimeout(3000),

                // === BUSCA LINHA 2 ===
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine2.ordinal())
                ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withTimeout(2000),
                new WaitUntilCommand(indexer::isFull).withTimeout(700),

                // === TIRO 4 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(600),
                new ShootCommandAutonomous(shooter, intake, indexer, indexer::getPieceCount).withTimeout(3000),

                // === TIRO 5 ===
                 /*new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine2.ordinal())
                ).withTimeout(2000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new ShootCommand(shooter, intake, indexer, 3, ledSubsystem).withTimeout(1200),*/

                // === FIM ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.EndPose.ordinal()))
        );
        addRequirements(drivetrain, shooter, intake);
    }
}