package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.PurePursuitConstants;
import org.firstinspires.ftc.teamcode.utils.PurePursuitController;
import org.firstinspires.ftc.teamcode.utils.Waypoint;

import java.util.ArrayList;

@TeleOp(name = "Pure Pursuit Tuner", group = "Tuning")
public class PurePursuitTuner extends LinearOpMode {

    private DrivetrainSubsystem drivetrain;
    private PurePursuitController controller;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;


    // PIDs
    private PIDController fwdPID, strPID, headPID;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        drivetrain = new DrivetrainSubsystem(hardwareMap, (com.bylazar.telemetry.TelemetryManager) null); // Null telemetry pq usamos a do OpMode

        // Inicializa PIDs com valores das constantes
        fwdPID = new PIDController(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
        strPID = new PIDController(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
        headPID = new PIDController(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

        controller = new PurePursuitController(fwdPID, strPID, headPID);

        // Caminho de Teste (Inicialmente vazio)
        ArrayList<Waypoint> testPath = new ArrayList<>();
        testPath.add(new Waypoint(0, 0, 0)); // Ponto dummy inicial

        waitForStart();

        while (opModeIsActive()) {
            // 1. ATUALIZAÇÃO EM TEMPO REAL (Dashboard -> PID)
            fwdPID.setPID(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
            strPID.setPID(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
            headPID.setPID(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

            // 2. CONTROLE PELO GAMEPAD
            if (gamepad1.a) {
                // Teste Forward: Vai para X=48
                testPath.clear();
                testPath.add(new Waypoint(0, 0, 0));
                testPath.add(new Waypoint(24, 0, 0)); // Waypoint intermediário
                testPath.add(new Waypoint(48, 0, 0));
                controller.setPath(testPath);
            }
            else if (gamepad1.b) {
                // Teste Strafe: Vai para Y=24
                testPath.clear();
                testPath.add(new Waypoint(0, 0, 0));
                testPath.add(new Waypoint(0, 24, 0));
                controller.setPath(testPath);
            }
            else if (gamepad1.y) {
                // Teste Turn: Gira 90 graus
                testPath.clear();
                testPath.add(new Waypoint(0, 0, 0));
                testPath.add(new Waypoint(0, 0, Math.toRadians(90)));
                controller.setPath(testPath);
            }
            else if (gamepad1.x) {
                // Reset: Volta para 0,0,0
                testPath.clear();
                testPath.add(new Waypoint(0, 0, 0));
                controller.setPath(testPath);
            }

            // 3. EXECUÇÃO DO PURE PURSUIT
            double x = drivetrain.getFollower().getPose().getX();
            double y = drivetrain.getFollower().getPose().getY();
            double h = drivetrain.getFollower().getPose().getHeading();

            double[] powers = controller.update(x, y, h);

            // Aplica limite de velocidade global para segurança durante testes
            for(int i=0; i<3; i++) {
                powers[i] = Math.max(-PurePursuitConstants.MAX_SPEED, Math.min(PurePursuitConstants.MAX_SPEED, powers[i]));
            }

            drivetrain.driveRobotCentric(powers[1], powers[0], powers[2]); // Strafe, Fwd, Turn
            drivetrain.periodic(); // Atualiza odometria e desenhos

            // 4. TELEMETRIA
            telemetry.addData("Mode", "TUNING");
            telemetry.addData("Target Path Size", testPath.size());
            telemetry.addData("X Error", fwdPID.getPositionError());
            telemetry.addData("Y Error", strPID.getPositionError());
            telemetry.addData("H Error", Math.toDegrees(headPID.getPositionError()));

            telemetry.addData("FWD Power", powers[0]);
            telemetry.addData("STR Power", powers[1]);
            telemetry.addData("TRN Power", powers[2]);
            telemetry.update();
        }
    }
}