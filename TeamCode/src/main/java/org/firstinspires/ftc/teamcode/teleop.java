package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.ActiveAimCommand;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.commands.KinematicAimDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

@TeleOp
public class teleop extends RobotOpMode {

    private AllianceEnum alliance;

    /** Comandos de botão do tipo "segurar": agendados ao pressionar, cancelados ao soltar. */
    private Command alignCommand;
    private Command kinematicAimCommand;
    private Command driverShootCommand;
    private Command operatorShootCommand;

    private long ultimoTempoTelemetria = 0;

    @Override
    public void start() {
        alliance = DataStorage.alliance;

        robot.applyTeleOpStartPose(alliance);

        // Comandos contínuos: prioridade base e SUSPEND, para que os comandos de botão os
        // suspendam e o escalonador os retome sozinho ao final.
        TeleOpDriveCommand.teleOpDrive(robot.drivetrain, gamepad1).schedule();

        double goalX = (alliance == AllianceEnum.Red) ? 130 : 14;
        double goalY = 130;
        ActiveAimCommand.activeAim(robot.shooter, robot.vision, robot.drivetrain,
                goalX, goalY, () -> robot.shooterAutoAdjust).schedule();

        Pose innitialPose = (alliance == AllianceEnum.Red)
                ? RedRearPoses.getPose(PosesNames.EndPose)
                : BlueRearPoses.getPose(PosesNames.EndPose);

        // Relocalização periódica: a cada 1 s, exceto mirando ou em movimento.
        Groups.loop(
                Groups.sequential(
                        Commands.waitMs(1000),
                        Commands.conditional(
                                () -> robot.isShooting() || !robot.drivetrain.isRobotStopped(),
                                Command.NOOP,
                                UpdatePoseLimelightCommand.updatePoseLimelight(
                                        robot.drivetrain, robot.vision, innitialPose)
                        )
                )
        ).schedule();

        Pose endPose = (alliance == AllianceEnum.Red)
                ? BlueRearPoses.getPose(PosesNames.EndPose)
                : RedRearPoses.getPose(PosesNames.EndPose);

        double targetx = (alliance == AllianceEnum.Red) ? 141 : 3;
        double targety = 144;

        alignCommand = AlignToAprilTagCommand.alignToAprilTag(
                robot.drivetrain, robot.vision, telemetryM, gamepad2);
        kinematicAimCommand = KinematicAimDriveCommand.kinematicAimDrive(
                robot.drivetrain, gamepad1, targetx, targety);
        driverShootCommand = AutoShootCommand.autoShoot(robot.drivetrain, robot.vision,
                robot.shooter, robot.intake, robot.indexer, endPose, gamepad1);
        operatorShootCommand = AutoShootCommand.autoShoot(robot.drivetrain, robot.vision,
                robot.shooter, robot.intake, robot.indexer, endPose, gamepad1);
    }

    @Override
    public void loop() {
        long tempoInicio = System.currentTimeMillis();

        bindDriver();
        bindOperator();

        super.loop();

        long tempoComandos = System.currentTimeMillis();

        if (DataStorage.DEBUG_MODE) {
            telemetryM.addData("1. Tempo do Cache (ms)", 0);
            telemetryM.addData("2. Tempo dos Comandos (ms)", tempoComandos - tempoInicio);
            telemetryM.addData("3. Tempo da Telemetria (ms)", ultimoTempoTelemetria);
        }

        long tempoTotal = (tempoComandos - tempoInicio) + ultimoTempoTelemetria;
        telemetryM.addData("⚡ TOTAL LOOP TIME (ms)", tempoTotal);

        long tempoAntesTelemetria = System.currentTimeMillis();
        telemetryM.update();
        ultimoTempoTelemetria = System.currentTimeMillis() - tempoAntesTelemetria;
    }

    /** Piloto (gamepad1). */
    private void bindDriver() {
        // Y (segurar): alinhar ao AprilTag
        if (gamepad1.yWasPressed()) alignCommand.schedule();
        if (gamepad1.yWasReleased()) alignCommand.cancel();

        // X (segurar): condução com mira cinemática
        if (gamepad1.xWasPressed()) kinematicAimCommand.schedule();
        if (gamepad1.xWasReleased()) kinematicAimCommand.cancel();

        // START: forçar relocalização por Limelight
        if (gamepad1.startWasPressed()) {
            UpdatePoseLimelightCommand.forceHardReset(robot.drivetrain, robot.vision, 90.0);
        }

        // LB (segurar): intake
        if (gamepad1.leftBumperWasPressed()) robot.intake.runCommand().schedule();
        if (gamepad1.leftBumperWasReleased()) robot.intake.stopCommand().schedule();

        // RB (segurar): macro de tiro
        if (gamepad1.rightBumperWasPressed()) driverShootCommand.schedule();
        if (gamepad1.rightBumperWasReleased()) driverShootCommand.cancel();
    }

    /** Operador (gamepad2). */
    private void bindOperator() {
        // RB (segurar): macro de tiro
        if (gamepad2.rightBumperWasPressed()) operatorShootCommand.schedule();
        if (gamepad2.rightBumperWasReleased()) operatorShootCommand.cancel();

        // LB (segurar): intake
        if (gamepad2.leftBumperWasPressed()) robot.intake.runCommand().schedule();
        if (gamepad2.leftBumperWasReleased()) robot.intake.stopCommand().schedule();

        // A (segurar): reverter intake
        if (gamepad2.aWasPressed()) robot.intake.reverseCommand().schedule();
        if (gamepad2.aWasReleased()) robot.intake.stopCommand().schedule();

        // X (segurar): motor de gatilho
        if (gamepad2.xWasPressed()) robot.intake.runTriggerCommand().schedule();
        if (gamepad2.xWasReleased()) robot.intake.stopCommand().schedule();

        // D-PAD ▼ / ▲: desliga e liga a mira automática
        if (gamepad2.dpadDownWasPressed()) {
            robot.shooterAutoAdjust = false;
            robot.shooter.stop();
        }
        if (gamepad2.dpadUpWasPressed()) robot.shooterAutoAdjust = true;

        // D-PAD ▶ / ◀: trim de RPM, com rumble curto
        if (gamepad2.dpadRightWasPressed()) {
            robot.shooter.adjustRpmOffset(10);
            gamepad2.rumble(100);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            robot.shooter.adjustRpmOffset(-10);
            gamepad2.rumble(100);
        }

        // Analógico esquerdo: zera o trim
        if (gamepad2.leftStickButtonWasPressed()) robot.shooter.resetRpmOffset();
    }
}
