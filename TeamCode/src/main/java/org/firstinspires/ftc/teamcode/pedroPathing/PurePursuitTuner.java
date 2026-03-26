package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.PurePursuitConstants;
import org.firstinspires.ftc.teamcode.utils.PurePursuitController;
import org.firstinspires.ftc.teamcode.utils.Waypoint;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "Pure Pursuit Tuner", group = "Tuning")
public class PurePursuitTuner extends LinearOpMode {

    private DrivetrainSubsystem drivetrain;
    private PurePursuitController controller;
    static TelemetryManager telemetryM;

    // PIDs
    private PIDController fwdPID, strPID, headPID;

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicializa Dashboard
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Passando telemetryM para evitar o NullPointerException!
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetryM);

        // Inicializa PIDs
        fwdPID = new PIDController(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
        strPID = new PIDController(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
        headPID = new PIDController(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

        controller = new PurePursuitController(fwdPID, strPID, headPID);

        // Define um caminho inicial (posição 0) para evitar erro de lista vazia
        ArrayList<Waypoint> initialPath = new ArrayList<>();
        initialPath.add(new Waypoint(0, 0, 90));
        controller.setPath(initialPath);

        // Força a pose inicial para garantir que não comece como null/NaN
        drivetrain.getFollower().setPose(new Pose(0, 0, 0));
        drivetrain.periodic();

        telemetry.addLine("Pronto para Tuning. Aperte Start.");
        telemetry.update();

        // --- VARIÁVEIS DE ESTADO PARA OS BOTÕES (Rising Edge) ---
        // Isso impede o "spam" de comandos enquanto o dedo estiver no botão
        boolean lastA = false;
        boolean lastB = false;
        boolean lastY = false;
        boolean lastX = false;

        waitForStart();

        while (opModeIsActive()) {
            // Atualiza Odometria PRIMEIRO
            drivetrain.periodic();

            // --- CORREÇÃO DO ERRO DE HEADING VAZIO ---
            Pose currentPose = drivetrain.getFollower().getPose();

            // Se for nulo (Pedro Pathing ainda a inicializar), pulamos este ciclo
            if (currentPose == null) {
                telemetryM.addData("Status", "A aguardar Odometria...");
                telemetryM.update();
                continue; // Volta para o início do while sem fazer contas
            }
            // -----------------------------------------

            // 1. ATUALIZAÇÃO PIDs (Dashboard)
            fwdPID.setPID(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
            strPID.setPID(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
            headPID.setPID(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

            // --- LENDO O ESTADO ATUAL DOS BOTÕES ---
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentY = gamepad1.y;
            boolean currentX = gamepad1.x;

            // 2. CONTROLE PELO GAMEPAD (Com proteção Rising Edge)
            // Só executa se o botão está pressionado AGORA e não estava ANTES
            if (currentA && !lastA) {
                ArrayList<Waypoint> p = new ArrayList<>();
                p.add(new Waypoint(0, 0, 0));
                p.add(new Waypoint(24, 0, 0));
                p.add(new Waypoint(48, 0, 0));
                controller.setPath(p);
            }
            else if (currentB && !lastB) {
                ArrayList<Waypoint> p = new ArrayList<>();
                p.add(new Waypoint(0, 0, 0));
                p.add(new Waypoint(0, 24, 0));
                controller.setPath(p);
            }
            else if (currentY && !lastY) {
                ArrayList<Waypoint> p = new ArrayList<>();
                p.add(new Waypoint(0, 0, 0));
                p.add(new Waypoint(0, 0, Math.toRadians(90)));
                controller.setPath(p);
            }
            else if (currentX && !lastX) {
                ArrayList<Waypoint> p = new ArrayList<>();
                p.add(new Waypoint(0, 0, 0)); // Reset
                controller.setPath(p);
            }

            // --- ATUALIZA O ESTADO ANTIGO PARA O PRÓXIMO CICLO ---
            lastA = currentA;
            lastB = currentB;
            lastY = currentY;
            lastX = currentX;

            // 3. EXECUÇÃO
            double x = currentPose.getX();
            double y = currentPose.getY();
            double h = currentPose.getHeading();

            double[] powers = controller.update(x, y, h);

            // Limite de segurança
            for(int i = 0; i < 3; i++) {
                powers[i] = Math.max(-PurePursuitConstants.MAX_SPEED, Math.min(PurePursuitConstants.MAX_SPEED, powers[i]));
            }

            // Enviando as potências para o chassi
            drivetrain.driveRobotCentric(powers[1], powers[0], powers[2]);

            // 4. TELEMETRIA
            telemetryM.addData("X", x);
            telemetryM.addData("Y", y);
            telemetryM.addData("Heading", Math.toDegrees(h));
            telemetryM.addData("X Error", fwdPID.getPositionError());
            telemetryM.addData("Y Error", strPID.getPositionError());
            telemetryM.addData("H Error", Math.toDegrees(headPID.getPositionError()));
            telemetryM.update();
        }
    }
}