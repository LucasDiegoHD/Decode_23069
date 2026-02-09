package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyConstants; // Seus arquivos de constantes
import org.firstinspires.ftc.teamcode.subsystems.HuskySubsystem;

import java.util.Optional;

public class ChaseArtifactCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final HuskySubsystem vision;
    private final PIDController turnPID, drivePID;

    public ChaseArtifactCommand(DrivetrainSubsystem drivetrain, HuskySubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // PIDs
        this.turnPID = new PIDController(HuskyConstants.TURN_KP, HuskyConstants.TURN_KI, HuskyConstants.TURN_KD);
        this.drivePID = new PIDController(HuskyConstants.DRIVE_KP, HuskyConstants.DRIVE_KI, HuskyConstants.DRIVE_KD);

        this.turnPID.setSetPoint(HuskyConstants.CENTER_X_PIXELS); // 160
        this.drivePID.setSetPoint(HuskyConstants.TARGET_DISTANCE_INCHES); // 8.0

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        // CRÍTICO: Reseta os PIDs ao apertar o botão para evitar "pulos" bruscos
        turnPID.reset();
        drivePID.reset();
    }

    @Override
    public void execute() {
        Optional<HuskyLens.Block> target = vision.getClosestAnyArtifact();

        if (target.isPresent()) {
            HuskyLens.Block block = target.get();
            double currentDist = vision.getDistanceToBlock(block);

            // Calcula PID
            double turnPower = turnPID.calculate(block.x);

            // ATENÇÃO: Se o robô for para TRÁS quando deveria ir para FRENTE,
            // remova o sinal negativo (-) abaixo.
            double drivePower = -drivePID.calculate(currentDist);

            // Clamp (Segurança para não acelerar demais no teste)
            turnPower = MathUtils.clamp(turnPower, -0.5, 0.5);
            drivePower = MathUtils.clamp(drivePower, -0.5, 0.5);

            // Feedforward (Força mínima para vencer o atrito do chão)
            if (Math.abs(drivePower) > 0.01 && Math.abs(drivePower) < 0.12) {
                drivePower = 0.12 * Math.signum(drivePower);
            }

            // Manda para o drive
            drivetrain.driveRobotCentric(drivePower, 0, turnPower);
        } else {
            // Se não vê nada, para imediatamente
            drivetrain.driveRobotCentric(0, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        // Retorna false para rodar "enquanto o botão estiver apertado"
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Garante que o robô pare quando você soltar o botão
        drivetrain.driveRobotCentric(0, 0, 0);
    }
}