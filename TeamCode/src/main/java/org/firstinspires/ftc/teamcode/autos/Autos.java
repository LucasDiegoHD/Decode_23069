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

    AllianceEnum selectedAlliance = AllianceEnum.Red;
    boolean isFront = false;

    // A variável mágica que diz se o piloto já confirmou a escolha
    boolean isConfigured = false;

    boolean xAnt = false, bAnt = false, upAnt = false, downAnt = false;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // =========================================================
        // FASE 1: MENU DE SELEÇÃO (Fica aqui até apertar 'A')
        // =========================================================
        while (!isStarted() && !isStopRequested() && !isConfigured) {

            if (gamepad1.x && !xAnt) selectedAlliance = AllianceEnum.Blue;
            if (gamepad1.b && !bAnt) selectedAlliance = AllianceEnum.Red;
            if (gamepad1.right_bumper && !upAnt) isFront = true;
            if (gamepad1.left_bumper && !downAnt) isFront = false;

            // O botão de CONFIRMAÇÃO (Lock-in)
            if (gamepad1.a) {
                isConfigured = true;
            }

            xAnt = gamepad1.x; bAnt = gamepad1.b; upAnt = gamepad1.dpad_up; downAnt = gamepad1.dpad_down;

            telemetry.addData("=== CONFIGURAÇÃO DO AUTÔNOMO ===", "");
            telemetry.addData("Aliança [X / B]", selectedAlliance == AllianceEnum.Red ? "🔴 VERMELHA" : "🔵 AZUL");
            telemetry.addData("Posição [RT / LT]", isFront ? "🔼 FRENTE (Triângulo Grande)" : "🔽 TRÁS (Triângulo Pequeno)");
            telemetry.addData("--------------------------------", "");
            telemetry.addData(">> APERTE 'A' PARA CONFIRMAR <<", "");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // =========================================================
        // FASE 2: INICIALIZAÇÃO DE HARDWARE (Ainda estamos no INIT!)
        // =========================================================
        telemetry.addData("Status", "⏳ Carregando Hardware... Não mexa no robô!");
        telemetry.update();

        DataStorage.alliance = selectedAlliance;
        RobotContainer robot = new RobotContainer(hardwareMap, telemetryM, null, null, selectedAlliance);
        Command autonomousCommand = null;

        // Pega o comando correto
        if (selectedAlliance == AllianceEnum.Red) {
            if (isFront) autonomousCommand = robot.getAutonomousRedFrontCommand();
            else autonomousCommand = robot.getAutonomousRedRearCommand();
        } else {
            if (isFront) autonomousCommand = robot.getAutonomousBlueFrontCommand();
            else autonomousCommand = robot.getAutonomousBlueRearCommand();
        }

        // =========================================================
        // FASE 3: TRAVA DE ODOMETRIA (Espera o PLAY sem enlouquecer o robô)
        // =========================================================
        while (!isStarted() && !isStopRequested()) {

            // Atualiza a posição inicial constantemente para o PedroPathing não derivar
            if (selectedAlliance == AllianceEnum.Red) {
                if (isFront) robot.updateRobotPose(AllianceEnum.Red, RedFrontPoses.getPose(PosesNames.StartPose));
                else robot.updateRobotPose(AllianceEnum.Red, RedRearPoses.getPose(PosesNames.StartPose));
            } else {
                if (isFront) robot.updateRobotPose(AllianceEnum.Blue, BlueFrontPoses.getPose(PosesNames.StartPose));
                else robot.updateRobotPose(AllianceEnum.Blue, BlueRearPoses.getPose(PosesNames.StartPose));
            }

            telemetry.addData("Status", "✅ PRONTO! PODE DAR O PLAY ▶️");
            telemetry.update();
        }

        // =========================================================
        // O JOGO COMEÇOU!
        // =========================================================
        if (autonomousCommand != null) {
            schedule(autonomousCommand);
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        if (telemetryM != null) {
            telemetryM.update();
        }
    }
}