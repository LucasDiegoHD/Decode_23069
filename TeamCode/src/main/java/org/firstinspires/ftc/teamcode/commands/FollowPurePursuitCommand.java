package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.utils.PurePursuitController;
import org.firstinspires.ftc.teamcode.utils.Waypoint;
import org.firstinspires.ftc.teamcode.utils.PurePursuitConstants;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.ArrayList;

/**
 * Command to follow a path using the custom Pure Pursuit controller.
 * Integration with PurePursuitConstants allows for real-time tuning via Dashboard.
 */
public class FollowPurePursuitCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final PurePursuitController controller;
    private final ArrayList<Waypoint> path;

    public FollowPurePursuitCommand(DrivetrainSubsystem drivetrain, ArrayList<Waypoint> path) {
        this.drivetrain = drivetrain;
        this.path = path;
        addRequirements(drivetrain);

        // --- PID INITIALIZATION USING CONSTANTS ---
        // Puxa os valores iniciais da classe de configuração
        PIDController fwdPID = new PIDController(
                PurePursuitConstants.FWD_P,
                PurePursuitConstants.FWD_I,
                PurePursuitConstants.FWD_D
        );

        PIDController strPID = new PIDController(
                PurePursuitConstants.STR_P,
                PurePursuitConstants.STR_I,
                PurePursuitConstants.STR_D
        );

        PIDController headPID = new PIDController(
                PurePursuitConstants.HEAD_P,
                PurePursuitConstants.HEAD_I,
                PurePursuitConstants.HEAD_D
        );

        this.controller = new PurePursuitController(fwdPID, strPID, headPID);
    }

    @Override
    public void initialize() {
        // Envia o caminho para o controlador
        controller.setPath(path);

        // Dica de Mentor: Você poderia atualizar os PIDs aqui novamente se quisesse
        // garantir que mudancas no dashboard afetem comandos ja instanciados,
        // mas geralmente mudar e reiniciar o OpMode é mais seguro.
    }

    @Override
    public void execute() {
        // 1. Get current pose from Pedro Pathing Odometry
        double x = drivetrain.getFollower().getPose().getX();
        double y = drivetrain.getFollower().getPose().getY();
        double heading = drivetrain.getFollower().getPose().getHeading();

        // 2. Run the Pure Pursuit Algorithm
        // Returns Robot Centric powers: [Forward, Strafe, Turn]
        double[] powers = controller.update(x, y, heading);

        // 3. Apply Speed Limits (Safety)
        double max = PurePursuitConstants.MAX_SPEED;

        // 4. Apply powers to the drivetrain
        // drivetrain.driveRobotCentric(Strafe, Forward, Turn)
        drivetrain.driveRobotCentric(
                Math.max(-max, Math.min(max, powers[1])), // Strafe (Index 1 do array do controller)
                Math.max(-max, Math.min(max, powers[0])), // Forward (Index 0)
                Math.max(-max, Math.min(max, powers[2]))  // Turn (Index 2)
        );
    }

    @Override
    public boolean isFinished() {
        double x = drivetrain.getFollower().getPose().getX();
        double y = drivetrain.getFollower().getPose().getY();
        return controller.isFinished(x, y);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}