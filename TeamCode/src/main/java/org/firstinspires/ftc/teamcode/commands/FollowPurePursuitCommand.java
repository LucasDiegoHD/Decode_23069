package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.PurePursuitConstants;
import org.firstinspires.ftc.teamcode.utils.PurePursuitController;
import org.firstinspires.ftc.teamcode.utils.Waypoint;

import java.util.ArrayList;

/**
 * Comando para seguir um caminho usando o controlador Pure Pursuit customizado.
 * Inclui proteção contra NullPointer, limites de segurança e Sistema Anti-Bloqueio.
 */
public class FollowPurePursuitCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final PurePursuitController controller;
    private final ArrayList<Waypoint> path;

    private final PIDController fwdPID;
    private final PIDController strPID;
    private final PIDController headPID;

    // --- TEMPORIZADORES DE SEGURANÇA ---
    private final ElapsedTime stuckTimer;
    private final ElapsedTime globalTimer;

    // Constantes de tempo (podes mover isto para o PurePursuitConstants depois)
    private final double MAX_STUCK_TIME = 1.0; // Abandona se ficar preso por 1 segundo
    private final double MAX_PATH_TIME = 15.0; // Abandona se o trajeto todo demorar mais de 15 segundos

    private boolean isStuck = false;

    public FollowPurePursuitCommand(DrivetrainSubsystem drivetrain, ArrayList<Waypoint> path) {
        this.drivetrain = drivetrain;
        this.path = path;
        addRequirements(drivetrain);

        fwdPID = new PIDController(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
        strPID = new PIDController(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
        headPID = new PIDController(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

        this.controller = new PurePursuitController(fwdPID, strPID, headPID);

        this.stuckTimer = new ElapsedTime();
        this.globalTimer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        fwdPID.setPID(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
        strPID.setPID(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
        headPID.setPID(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

        controller.setPath(path);

        // Reinicia os relógios quando o caminho começa
        stuckTimer.reset();
        globalTimer.reset();
        isStuck = false;
    }

    @Override
    public void execute() {
        Pose currentPose = drivetrain.getFollower().getPose();

        if (currentPose == null) {
            drivetrain.stop();
            return;
        }

        // --- LÓGICA ANTI-BLOQUEIO (Stuck Detection) ---
        // Se o robô estiver a movimentar-se livremente, o relógio de bloqueio é constantemente zerado.
        if (!drivetrain.isRobotStopped()) {
            stuckTimer.reset();
        } else if (stuckTimer.seconds() > MAX_STUCK_TIME) {
            // Se o relógio não foi zerado num período de 1 segundo, o robô está preso.
            isStuck = true;
        }

        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading = currentPose.getHeading();

        double[] powers = controller.update(x, y, heading);
        double max = PurePursuitConstants.MAX_SPEED;

        drivetrain.driveRobotCentric(
                Math.max(-max, Math.min(max, powers[1])),
                Math.max(-max, Math.min(max, powers[0])),
                Math.max(-max, Math.min(max, powers[2]))
        );
    }

    @Override
    public boolean isFinished() {
        // 1. O robô demorou demasiado tempo no total? (Falha de segurança)
        if (globalTimer.seconds() > MAX_PATH_TIME) return true;

        // 2. O robô está preso a empurrar uma parede? (Falha de segurança)
        if (isStuck) return true;

        // 3. Chegámos ao destino normalmente? (Cenário ideal)
        Pose currentPose = drivetrain.getFollower().getPose();
        if (currentPose == null) return false;

        return controller.isFinished(currentPose.getX(), currentPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        // Opcional: Aqui poderias enviar um alerta para a telemetria a avisar que a trajetória foi abortada!
    }
}