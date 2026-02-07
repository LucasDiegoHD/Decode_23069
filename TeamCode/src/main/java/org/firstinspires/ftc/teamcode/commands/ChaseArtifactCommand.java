package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HuskyConstants;
import org.firstinspires.ftc.teamcode.subsystems.HuskySubsystem;

import java.util.Optional;

public class ChaseArtifactCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final HuskySubsystem vision;

    private final PIDController turnPID;
    private final PIDController drivePID;

    public ChaseArtifactCommand(DrivetrainSubsystem drivetrain, HuskySubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Mesmos ganhos PID do VisionConstants
        this.turnPID = new PIDController(HuskyConstants.TURN_KP, HuskyConstants.TURN_KI, HuskyConstants.TURN_KD);
        this.drivePID = new PIDController(HuskyConstants.DRIVE_KP, HuskyConstants.DRIVE_KI, HuskyConstants.DRIVE_KD);

        this.turnPID.setSetPoint(HuskyConstants.CENTER_X_PIXELS); // Alvo: X=160
        this.drivePID.setSetPoint(HuskyConstants.TARGET_DISTANCE_INCHES); // Alvo: 8 polegadas

        addRequirements(drivetrain, vision);
    }

    @Override
    public void execute() {
        // CHAMA O NOVO MÉTODO "ANY" (QUALQUER COR)
        Optional<HuskyLens.Block> target = vision.getClosestAnyArtifact();

        if (target.isPresent()) {
            HuskyLens.Block block = target.get();
            double currentDist = vision.getDistanceToBlock(block);

            // Cálculo dos PIDs
            double turnPower = turnPID.calculate(block.x);
            double drivePower = -drivePID.calculate(currentDist); // Negativo pois erro negativo = longe = avançar

            // Travas de segurança (Clamp)
            turnPower = Math.max(-HuskyConstants.MAX_TURN_SPEED, Math.min(HuskyConstants.MAX_TURN_SPEED, turnPower));
            drivePower = Math.max(-HuskyConstants.MAX_DRIVE_SPEED, Math.min(HuskyConstants.MAX_DRIVE_SPEED, drivePower));

            if (Math.abs(turnPower) > 0.01 && Math.abs(turnPower) < 0.15) {
                turnPower = 0.15 * Math.signum(turnPower);
            }

            // Aplica movimento
            drivetrain.driveRobotCentric(drivePower, 0, turnPower);

        } else {
            // Se não ver nada, para (ou você pode colocar para girar devagar buscando)
            drivetrain.driveRobotCentric(0, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        // Comando contínuo (enquanto segura o botão). Retorne true se quiser que pare automático ao chegar.
        return false;
    }
}