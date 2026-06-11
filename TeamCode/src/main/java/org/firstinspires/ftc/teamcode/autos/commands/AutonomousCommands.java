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
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
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
                new SpinShooterCommand(shooter, SpinShooterCommand.Action.LONG_SHOOT),

                // === TIRO 1 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(1000),
                new WaitUntilCommand(shooter::getShooterAtTarget).withTimeout(700),
                new ShootCommandAutonomous(shooter, intake, indexer, 2).withTimeout(3000),

                // === BUSCA LINHA 1 ===
                new ParallelCommandGroup(
                        new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine1.ordinal()),
                        poses.get(PosesNames.CatchLine1.ordinal())
                        ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withConstantHeading().withTimeout(4000),
                        new InstantCommand(intake::run)
                ),

                // === TIRO 2 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoShootConstraints).withConstantHeading(),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(500),
                new ShootCommandAutonomous(shooter, intake, indexer, 2).withTimeout(3000),

                // === BUSCA LINHA 3 ===
                new ParallelCommandGroup(
                        new GoToPoseCommand(drivetrain, true,
                                poses.get(PosesNames.GoToLine3.ordinal()),
                                poses.get(PosesNames.CatchLine3.ordinal())
                        ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withConstantHeading().withTimeout(4000),
                        new InstantCommand(intake::run)
                        ),

                // === GATE ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GatePose.ordinal()))
                        .setConstraints(Constants.autoShootConstraints).withTimeout(800),

                // === TIRO 3 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal()))
                        .setConstraints(Constants.autoTransitConstraints).withConstantHeading(),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(500),
                new ShootCommandAutonomous(shooter, intake, indexer, 2).withTimeout(3000),

                // === BUSCA LINHA 2 ===
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine2.ordinal())
                ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withConstantHeading().withTimeout(2000),

                // === TIRO 4 ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot2.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new AlignToAprilTagCommand(drivetrain, vision, PanelsTelemetry.INSTANCE.getTelemetry(), null).withTimeout(800),
                new ShootCommandAutonomous(shooter, intake, indexer, 2).withTimeout(3000),

                // === TIRO 5 ===
                new ParallelCommandGroup(
                        new GoToPoseCommand(drivetrain, true,
                                poses.get(PosesNames.GoToLine2.ordinal()),
                                poses.get(PosesNames.CatchLine2.ordinal())
                        ).setConstraints(Constants.autoTransitConstraints).withNoDeceleration().withConstantHeading().withTimeout(4000),
                        new InstantCommand(intake::run)
                ),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot2.ordinal()))
                        .setConstraints(Constants.autoShootConstraints),
                new ShootCommandAutonomous(shooter, intake, indexer, 2).withTimeout(2400),

                // === FIM ===
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.EndPose.ordinal()))
        );
        addRequirements(drivetrain, shooter, intake);
    }
}