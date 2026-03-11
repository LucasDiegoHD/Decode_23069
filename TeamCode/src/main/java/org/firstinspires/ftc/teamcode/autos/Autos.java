package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.BlueFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

@Autonomous(name = "⭐️ Autônomo Geral (Seletor)", group = "Competição")
public class Autos extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    enum Strategy { FRONT, REAR_NORMAL, REAR_NO_GATE }

    AllianceEnum selectedAlliance = AllianceEnum.Red;
    Strategy selectedStrategy = Strategy.REAR_NORMAL;

    boolean isConfigured = false;

    boolean xAnt = false, bAnt = false;
    boolean upAnt = false, downAnt = false, rightAnt = false, leftAnt = false;
    private RobotContainer robot;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        while (!isStarted() && !isStopRequested() && !isConfigured) {

            if (gamepad2.x && !xAnt) selectedAlliance = AllianceEnum.Blue;
            if (gamepad2.b && !bAnt) selectedAlliance = AllianceEnum.Red;

            if (gamepad2.dpad_up && !upAnt) selectedStrategy = Strategy.FRONT;
            if (gamepad2.dpad_down && !downAnt) selectedStrategy = Strategy.REAR_NORMAL;
            if (gamepad2.dpad_right && !rightAnt) selectedStrategy = Strategy.REAR_NO_GATE;
            if (gamepad2.dpad_left && !leftAnt) selectedStrategy = Strategy.REAR_NORMAL;

            if (gamepad2.a) {
                isConfigured = true;
            }

            xAnt = gamepad2.x; bAnt = gamepad2.b;
            upAnt = gamepad2.dpad_up; downAnt = gamepad2.dpad_down;
            rightAnt = gamepad2.dpad_right; leftAnt = gamepad2.dpad_left;

            telemetry.addData("=== CONFIGURAÇÃO DO AUTÔNOMO ===", "");
            telemetry.addData("Aliança [X / B]", selectedAlliance == AllianceEnum.Red ? "🔴 VERMELHA" : "🔵 AZUL");
            String stratText = "";
            if (selectedStrategy == Strategy.FRONT) stratText = "🔼 FRENTE (Triângulo Grande)";
            else if (selectedStrategy == Strategy.REAR_NORMAL) stratText = "🔽/◀️ TRÁS (Com Gate / Padrão)";
            else if (selectedStrategy == Strategy.REAR_NO_GATE) stratText = "▶️ TRÁS (Sem Gate / 15 Artefatos)";

            telemetry.addData("Posição [Setas D-PAD]", stratText);
            telemetry.addData("--------------------------------", "");
            telemetry.addData(">> APERTE 'A' PARA CONFIRMAR <<", "");
            telemetry.update();
        }

        if (isStopRequested()) return;

        telemetry.addData("Status", "⏳ Carregando Hardware... Não mexa no robô!");
        telemetry.update();

        DataStorage.alliance = selectedAlliance;
        robot = new RobotContainer(hardwareMap, telemetryM, null, null, selectedAlliance);
        Command autonomousCommand = null;

        if (selectedAlliance == AllianceEnum.Red) {
            if (selectedStrategy == Strategy.FRONT) {
                autonomousCommand = robot.getAutonomousRedFrontCommand();
            } else if (selectedStrategy == Strategy.REAR_NORMAL) {
                autonomousCommand = robot.getAutonomousRedRearCommand();
            } else if (selectedStrategy == Strategy.REAR_NO_GATE) {
                autonomousCommand = robot.getAutonomousRedTuffCommand();
            }
        } else {
            if (selectedStrategy == Strategy.FRONT) {
                autonomousCommand = robot.getAutonomousBlueFrontCommand();
            } else if (selectedStrategy == Strategy.REAR_NORMAL) {
                autonomousCommand = robot.getAutonomousBlueRearCommand();
            } else if (selectedStrategy == Strategy.REAR_NO_GATE) {
                autonomousCommand = robot.getAutonomousBlueTuffCommand();
            }
        }

        while (!isStarted() && !isStopRequested()) {

            if (selectedAlliance == AllianceEnum.Red) {
                if (selectedStrategy == Strategy.FRONT) robot.updateRobotPose(AllianceEnum.Red, RedFrontPoses.getPose(PosesNames.StartPose));
                else robot.updateRobotPose(AllianceEnum.Red, RedRearPoses.getPose(PosesNames.StartPose));
            } else {
                if (selectedStrategy == Strategy.FRONT) robot.updateRobotPose(AllianceEnum.Blue, BlueFrontPoses.getPose(PosesNames.StartPose));
                else robot.updateRobotPose(AllianceEnum.Blue, BlueRearPoses.getPose(PosesNames.StartPose));
            }

            telemetry.addData("Status", "✅ PRONTO! PODE DAR O PLAY ▶️");
            telemetry.update();
        }

        if (autonomousCommand != null) {
            schedule(autonomousCommand);
        }
    }

    @Override
    public void run() {
        if (robot != null) {
            robot.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        if (telemetryM != null) {
            telemetryM.update();
        }
    }
}