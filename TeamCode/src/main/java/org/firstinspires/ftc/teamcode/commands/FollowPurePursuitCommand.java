package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.PurePursuitConstants;
import org.firstinspires.ftc.teamcode.utils.PurePursuitController;
import org.firstinspires.ftc.teamcode.utils.Waypoint;

import java.util.ArrayList;

/**
 * Comando para seguir um caminho usando o controlador Pure Pursuit customizado.
 * Inclui proteção contra NullPointer (caso a odometria falhe) e limites de segurança.
 */
public class FollowPurePursuitCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final PurePursuitController controller;
    private final ArrayList<Waypoint> path;

    // Controladores PID locais
    private final PIDController fwdPID;
    private final PIDController strPID;
    private final PIDController headPID;

    public FollowPurePursuitCommand(DrivetrainSubsystem drivetrain, ArrayList<Waypoint> path) {
        this.drivetrain = drivetrain;
        this.path = path;
        addRequirements(drivetrain);

        // Inicializa os PIDs usando as constantes globais
        // Nota: Se você mudar as constantes no Dashboard DURANTE o init,
        // estes valores serão atualizados no método initialize()
        fwdPID = new PIDController(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
        strPID = new PIDController(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
        headPID = new PIDController(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

        this.controller = new PurePursuitController(fwdPID, strPID, headPID);
    }

    @Override
    public void initialize() {
        // --- ATUALIZAÇÃO DE TUNING ---
        // Recarrega os valores do Dashboard na hora que o comando começa.
        // Isso permite tunar sem reiniciar o OpMode se você reiniciar apenas o comando.
        fwdPID.setPID(PurePursuitConstants.FWD_P, PurePursuitConstants.FWD_I, PurePursuitConstants.FWD_D);
        strPID.setPID(PurePursuitConstants.STR_P, PurePursuitConstants.STR_I, PurePursuitConstants.STR_D);
        headPID.setPID(PurePursuitConstants.HEAD_P, PurePursuitConstants.HEAD_I, PurePursuitConstants.HEAD_D);

        controller.setPath(path);
    }

    @Override
    public void execute() {
        // --- BLINDAGEM CONTRA NULL POINTER ---
        // Obtém a pose atual do Pedro Pathing
        Pose currentPose = drivetrain.getFollower().getPose();

        if (currentPose == null) {
            // Se o robô não sabe onde está (odometria iniciando), paramos por segurança
            // e retornamos para tentar no próximo ciclo.
            drivetrain.stop();
            return;
        }
        // -------------------------------------

        double x = currentPose.getX();
        double y = currentPose.getY();
        double heading = currentPose.getHeading();

        // Roda o algoritmo Pure Pursuit
        // Retorna potências Robot Centric: [Forward, Strafe, Turn]
        double[] powers = controller.update(x, y, heading);

        // Aplica limite de velocidade global (Segurança)
        double max = PurePursuitConstants.MAX_SPEED;

        // Limita as potências e envia para o Drivetrain
        // drivetrain.driveRobotCentric(Strafe, Forward, Turn)
        drivetrain.driveRobotCentric(
                Math.max(-max, Math.min(max, powers[1])), // powers[1] é Strafe
                Math.max(-max, Math.min(max, powers[0])), // powers[0] é Forward
                Math.max(-max, Math.min(max, powers[2]))  // powers[2] é Turn
        );
    }

    @Override
    public boolean isFinished() {
        Pose currentPose = drivetrain.getFollower().getPose();

        // Se a pose for nula, não podemos saber se chegamos, então retornamos false
        if (currentPose == null) return false;

        return controller.isFinished(currentPose.getX(), currentPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}