package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.BlueFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Seletor de autônomo: aliança e estratégia escolhidas antes do play.
 *
 * <p>A configuração roda no {@link #init_loop()}, não num laço bloqueante: a base
 * {@link RobotOpMode} é iterativa, então o escalonador segue avançando durante a espera e a
 * odometria assenta antes do play. Nada é agendado até o {@link #start()}.
 */
@Autonomous(name = "⭐️ Autônomo Geral (Seletor)", group = "Competição")
public class Autos extends RobotOpMode {

    enum Strategy { FRONT, REAR_NORMAL, REAR_NO_GATE }

    private AllianceEnum selectedAlliance = AllianceEnum.Red;
    private Strategy selectedStrategy = Strategy.REAR_NORMAL;
    private boolean isConfigured = false;

    private Command autonomousCommand;

    @Override
    public void init_loop() {
        if (!isConfigured) {
            readConfiguration();
            showConfiguration();
        }

        super.init_loop();
    }

    /** Lê o gamepad2 e atualiza a seleção. `A` confirma e prepara a rotina. */
    private void readConfiguration() {
        if (gamepad2.xWasPressed()) selectedAlliance = AllianceEnum.Blue;
        if (gamepad2.bWasPressed()) selectedAlliance = AllianceEnum.Red;

        if (gamepad2.dpadUpWasPressed()) selectedStrategy = Strategy.FRONT;
        if (gamepad2.dpadDownWasPressed()) selectedStrategy = Strategy.REAR_NORMAL;
        if (gamepad2.dpadLeftWasPressed()) selectedStrategy = Strategy.REAR_NORMAL;
        if (gamepad2.dpadRightWasPressed()) selectedStrategy = Strategy.REAR_NO_GATE;

        if (gamepad2.a) {
            isConfigured = true;
            prepareRoutine();
        }
    }

    private void showConfiguration() {
        telemetry.addData("=== CONFIGURAÇÃO DO AUTÔNOMO ===", "");
        telemetry.addData("Aliança [X / B]",
                selectedAlliance == AllianceEnum.Red ? "🔴 VERMELHA" : "🔵 AZUL");

        String stratText = "";
        if (selectedStrategy == Strategy.FRONT) stratText = "🔼 FRENTE (Triângulo Grande)";
        else if (selectedStrategy == Strategy.REAR_NORMAL) stratText = "🔽/◀️ TRÁS (Com Gate / Padrão)";
        else if (selectedStrategy == Strategy.REAR_NO_GATE) stratText = "▶️ TRÁS (Sem Gate / 15 Artefatos)";

        telemetry.addData("Posição [Setas D-PAD]", stratText);
        telemetry.addData("--------------------------------", "");
        telemetry.addData(">> APERTE 'A' PARA CONFIRMAR <<", "");
        telemetry.update();
    }

    /** Escolhe rotina e pose inicial pela combinação aliança × estratégia. */
    private void prepareRoutine() {
        DataStorage.alliance = selectedAlliance;

        Pose startPose;

        if (selectedAlliance == AllianceEnum.Red) {
            switch (selectedStrategy) {
                case FRONT:
                    autonomousCommand = AutoRoutines.front(robot, RedFrontPoses.asList());
                    startPose = RedFrontPoses.getPose(PosesNames.StartPose);
                    break;
                case REAR_NO_GATE:
                    autonomousCommand = AutoRoutines.rearNoGate(robot, RedRearPoses.asList());
                    startPose = RedRearPoses.getPose(PosesNames.StartPose);
                    break;
                case REAR_NORMAL:
                default:
                    autonomousCommand = AutoRoutines.rearNormal(robot, RedRearPoses.asList());
                    startPose = RedRearPoses.getPose(PosesNames.StartPose);
                    break;
            }
        } else {
            switch (selectedStrategy) {
                case FRONT:
                    autonomousCommand = AutoRoutines.front(robot, BlueFrontPoses.asList());
                    startPose = BlueFrontPoses.getPose(PosesNames.StartPose);
                    break;
                case REAR_NO_GATE:
                    autonomousCommand = AutoRoutines.rearNoGate(robot, BlueRearPoses.asList());
                    startPose = BlueRearPoses.getPose(PosesNames.StartPose);
                    break;
                case REAR_NORMAL:
                default:
                    autonomousCommand = AutoRoutines.rearNormal(robot, BlueRearPoses.asList());
                    startPose = BlueRearPoses.getPose(PosesNames.StartPose);
                    break;
            }
        }

        robot.setAutoStartPose(startPose);

        telemetry.addData("Status", "✅ Pinpoint Rastreando! PRONTO! PODE DAR O PLAY ▶️");
        telemetry.update();
    }

    @Override
    public void start() {
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }
}
