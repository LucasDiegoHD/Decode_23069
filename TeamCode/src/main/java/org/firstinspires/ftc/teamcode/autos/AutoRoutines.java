package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.autos.commands.GoToPoseCommand;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommands;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

/**
 * As três rotinas de autônomo, como composições do Ivy.
 *
 * <p>Substitui as classes {@code AutonomousCommands}, {@code AutonomousFrontCommands} e
 * {@code AutonomousTuffCommand}, que eram {@code SequentialCommandGroup} da FTCLib. A tradução é
 * 1:1: cada {@code addCommands(...)} virou {@link Groups#sequential}, cada
 * {@code ParallelCommandGroup} virou {@link Groups#parallel}, e cada {@code .withTimeout(t)}
 * virou uma corrida contra {@link Commands#waitMs}.
 *
 * <p>O parâmetro {@code LEDSubsystem} das versões antigas foi removido — nenhuma delas o usava.
 */
public final class AutoRoutines {

    private AutoRoutines() {
    }

    /** Corta um comando no tempo, como o antigo {@code .withTimeout(ms)} da FTCLib. */
    private static Command withTimeout(Command command, double milliseconds) {
        return command.raceWith(Commands.waitMs(milliseconds));
    }

    /** Alinha ao AprilTag com limite de tempo. Sem controle para vibrar, no autônomo. */
    private static Command align(Robot robot, double timeoutMs) {
        return withTimeout(
                AlignToAprilTagCommand.alignToAprilTag(robot.drivetrain, robot.vision,
                        PanelsTelemetry.INSTANCE.getTelemetry(), null),
                timeoutMs);
    }

    private static Command shoot(Robot robot, int pieces, double timeoutMs) {
        return withTimeout(
                ShootCommand.shoot(robot.shooter, robot.intake, robot.indexer, pieces),
                timeoutMs);
    }

    private static Command adjust(Robot robot) {
        return ShooterCommands.alignAndAdjustAuto(robot.shooter, robot.vision);
    }

    private static Command relocalize(Robot robot, Pose fallback) {
        return UpdatePoseLimelightCommand.updatePoseLimelight(robot.drivetrain, robot.vision, fallback);
    }

    private static Pose pose(List<Pose> poses, PosesNames name) {
        return poses.get(name.ordinal());
    }

    /** Traseira com gate: ~5 ciclos de tiro intercalados com coletas de linha. */
    public static Command rearNormal(Robot robot, List<Pose> poses) {
        return Groups.sequential(
                // === INÍCIO ===
                relocalize(robot, pose(poses, PosesNames.StartPose)),
                adjust(robot),
                ShooterCommands.spin(robot.shooter, ShooterCommands.Action.LONG_SHOOT),

                // === TIRO 1 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                relocalize(robot, pose(poses, PosesNames.GoToShoot1)),
                adjust(robot),
                align(robot, 1000),
                withTimeout(Commands.waitUntil(robot.shooter::getShooterAtTarget), 700),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 1 ===
                Groups.parallel(
                        withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                                pose(poses, PosesNames.GoToLine1),
                                pose(poses, PosesNames.CatchLine1))
                                .setConstraints(Constants.autoTransitConstraints)
                                .withNoDeceleration()
                                .withConstantHeading()
                                .toCommand(), 4000),
                        Commands.instant(robot.intake::run)
                ),

                // === TIRO 2 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .withConstantHeading()
                        .toCommand(),
                align(robot, 500),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 3 ===
                Groups.parallel(
                        withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                                pose(poses, PosesNames.GoToLine3),
                                pose(poses, PosesNames.CatchLine3))
                                .setConstraints(Constants.autoTransitConstraints)
                                .withNoDeceleration()
                                .withConstantHeading()
                                .toCommand(), 4000),
                        Commands.instant(robot.intake::run)
                ),

                // === GATE ===
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GatePose))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(), 800),

                // === TIRO 3 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withConstantHeading()
                        .toCommand(),
                align(robot, 500),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 2 ===
                Commands.instant(robot.intake::run),
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        pose(poses, PosesNames.GoToLine2),
                        pose(poses, PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 2000),

                // === TIRO 4 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                align(robot, 800),
                shoot(robot, 2, 3000),

                // === TIRO 5 ===
                Groups.parallel(
                        withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                                pose(poses, PosesNames.GoToLine2),
                                pose(poses, PosesNames.CatchLine2))
                                .setConstraints(Constants.autoTransitConstraints)
                                .withNoDeceleration()
                                .withConstantHeading()
                                .toCommand(), 4000),
                        Commands.instant(robot.intake::run)
                ),
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                shoot(robot, 2, 2400),

                // === FIM ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.EndPose)).toCommand()
        );
    }

    /** Frente: triângulo grande, 4 ciclos de 3 peças. */
    public static Command front(Robot robot, List<Pose> poses) {
        return Groups.sequential(
                relocalize(robot, pose(poses, PosesNames.StartPose)),
                ShooterCommands.spin(robot.shooter, ShooterCommands.Action.SHORT_SHOOT),
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .toCommand(), 1500),
                withTimeout(Commands.waitUntil(robot.shooter::isReady), 800),
                adjust(robot),
                shoot(robot, 3, 3000),
                ShooterCommands.spin(robot.shooter, ShooterCommands.Action.SHORT_SHOOT),
                Commands.instant(robot.intake::run),
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        pose(poses, PosesNames.GoToLine2),
                        pose(poses, PosesNames.CatchLine2)).toCommand(), 4000),
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2)).toCommand(),
                adjust(robot),
                Commands.waitMs(800),
                shoot(robot, 3, 3000),
                ShooterCommands.spin(robot.shooter, ShooterCommands.Action.SHORT_SHOOT),
                Commands.instant(robot.intake::run),
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        pose(poses, PosesNames.GoToLine1),
                        pose(poses, PosesNames.CatchLine1)).toCommand(), 4000),
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2))
                        .toCommand(), 2000),
                Commands.waitMs(800),
                adjust(robot),
                shoot(robot, 3, 3000),
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GatePose))
                        .toCommand(), 2000),
                Commands.instant(robot.intake::run),
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.CatchLine3))
                        .toCommand(), 2000),
                Commands.waitMs(600),
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .toCommand(), 2000),
                Commands.waitMs(800),
                shoot(robot, 3, 3000),
                withTimeout(new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.EndPose))
                        .toCommand(), 2000),
                ShooterCommands.spin(robot.shooter, ShooterCommands.Action.STOP)
        );
    }

    /** Traseira sem gate: 5 ciclos de tiro, 15 artefatos. */
    public static Command rearNoGate(Robot robot, List<Pose> poses) {
        return Groups.sequential(
                // === INÍCIO ===
                relocalize(robot, pose(poses, PosesNames.StartPose)),
                ShooterCommands.spin(robot.shooter, ShooterCommands.Action.LONG_SHOOT),
                adjust(robot),

                // === TIRO 1 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                relocalize(robot, pose(poses, PosesNames.GoToShoot1)),
                adjust(robot),
                withTimeout(Commands.waitUntil(robot.shooter::getShooterAtTarget), 1700),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 1 ===
                Groups.parallel(
                        withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                                pose(poses, PosesNames.GoToLine1),
                                pose(poses, PosesNames.CatchLine1))
                                .setConstraints(Constants.autoTransitConstraints)
                                .withNoDeceleration()
                                .withConstantHeading()
                                .toCommand(), 3000),
                        Commands.instant(robot.intake::run)
                ),

                // === TIRO 2 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot1))
                        .setConstraints(Constants.autoShootConstraints)
                        .withConstantHeading()
                        .toCommand(),
                adjust(robot),
                align(robot, 500),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 2 ===
                Commands.instant(robot.intake::run),
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        pose(poses, PosesNames.GoToLine2),
                        pose(poses, PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 2000),

                // === TIRO 3 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                align(robot, 600),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 3 ===
                Commands.instant(robot.intake::run),
                withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                        pose(poses, PosesNames.GoToLine2),
                        pose(poses, PosesNames.CatchLine2))
                        .setConstraints(Constants.autoTransitConstraints)
                        .withNoDeceleration()
                        .withConstantHeading()
                        .toCommand(), 2000),

                // === TIRO 4 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .toCommand(),
                align(robot, 600),
                shoot(robot, 2, 3000),

                // === BUSCA LINHA 4 ===
                Groups.parallel(
                        withTimeout(new GoToPoseCommand(robot.drivetrain, true,
                                pose(poses, PosesNames.GoToLine1),
                                pose(poses, PosesNames.CatchLine1))
                                .setConstraints(Constants.autoTransitConstraints)
                                .withNoDeceleration()
                                .withConstantHeading()
                                .toCommand(), 4000),
                        Commands.instant(robot.intake::run)
                ),

                // === TIRO 5 ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.GoToShoot2))
                        .setConstraints(Constants.autoShootConstraints)
                        .withConstantHeading()
                        .toCommand(),
                align(robot, 500),
                shoot(robot, 2, 3000),

                // === FIM ===
                new GoToPoseCommand(robot.drivetrain, pose(poses, PosesNames.EndPose)).toCommand()
        );
    }
}
