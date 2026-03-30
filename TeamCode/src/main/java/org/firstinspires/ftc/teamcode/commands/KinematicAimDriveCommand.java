package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;

public class KinematicAimDriveCommand extends CommandBase {

    private final Follower follower;
    private final PIDFController turnController;
    private final GamepadEx driver;
    private final double targetX;
    private final double targetY;

    private static final double ARTIFACT_VELOCITY_INCHES_PER_SEC = 400.0;
    private static final double SYSTEM_LATENCY_SECONDS = 0.20;

    // --- VARIÁVEIS DO FILTRO DE FLUIDEZ ---
    private double smoothedVelX = 0.0;
    private double smoothedVelY = 0.0;
    private static final double VEL_ALPHA = 0.5; // Quanto menor, mais suave (mas demora mais pra reagir). 0.25 é o ponto doce.

    public KinematicAimDriveCommand(DrivetrainSubsystem drivetrain, GamepadEx driver, double targetX, double targetY) {
        this.follower = drivetrain.getFollower();
        this.driver = driver;
        this.targetX = targetX;
        this.targetY = targetY;

        turnController = new PIDFController(
                ShooterConstants.ANGLE_KP,
                ShooterConstants.ANGLE_KI,
                ShooterConstants.ANGLE_KD,
                ShooterConstants.ANGLE_KF
        );

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        turnController.reset();
        turnController.setSetPoint(0);
        turnController.setTolerance((Math.PI / 180.0) * ShooterConstants.ANGLE_TOLERANCE);

        // Zera o filtro ao iniciar o comando
        smoothedVelX = follower.getVelocity().getXComponent();
        smoothedVelY = follower.getVelocity().getYComponent();
    }

    @Override
    public void execute() {
        // Leitura do piloto (Mantive a sua inversão de X e Y)
        double forward = -driver.getLeftX();
        double strafe = -driver.getLeftY();

        // --- INVERSÃO FIELD-ORIENTED (Lado Azul) ---
        if (org.firstinspires.ftc.teamcode.utils.DataStorage.alliance == org.firstinspires.ftc.teamcode.utils.AllianceEnum.Blue) {
            forward = -forward;
            strafe = -strafe;
        }

        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        // --- FILTRO PASSA-BAIXA (A "Manteiga") ---
        // Pega a tremedeira do sensor e transforma num deslize perfeito
        smoothedVelX = (VEL_ALPHA * velocity.getXComponent()) + ((1 - VEL_ALPHA) * smoothedVelX);
        smoothedVelY = (VEL_ALPHA * velocity.getYComponent()) + ((1 - VEL_ALPHA) * smoothedVelY);

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        double diffX = targetX - robotX;
        double diffY = targetY - robotY;
        double distanceToTarget = Math.hypot(diffX, diffY);
        if (distanceToTarget < 1.0) distanceToTarget = 1.0;

        // --- VELOCIDADE DINÂMICA DA BOLA (Doppler) ---
        double targetDirX = diffX / distanceToTarget;
        double targetDirY = diffY / distanceToTarget;
        double velTowardsGoal = (smoothedVelX * targetDirX) + (smoothedVelY * targetDirY);

        double effectiveArtifactVelocity = ARTIFACT_VELOCITY_INCHES_PER_SEC + velTowardsGoal;
        if (effectiveArtifactVelocity < 100.0) effectiveArtifactVelocity = 100.0;

        double timeOfFlight = distanceToTarget / effectiveArtifactVelocity;
        double totalPredictionTime = timeOfFlight + SYSTEM_LATENCY_SECONDS;

        // Usamos a velocidade FILTRADA para o alvo virtual!
        double virtualX = targetX - (smoothedVelX * totalPredictionTime);
        double virtualY = targetY - (smoothedVelY * totalPredictionTime);

        double desiredAngle = Math.atan2(virtualY - robotY, virtualX - robotX);
        double error = angleDifference(desiredAngle, heading);

        turnController.setPIDF(ShooterConstants.ANGLE_KP, ShooterConstants.ANGLE_KI, ShooterConstants.ANGLE_KD, ShooterConstants.ANGLE_KF);
        double turnPower = turnController.calculate(error);

        // --- PROTEÇÃO ANTI-VIBRAÇÃO DO kF ---
        double toleranceRads = (Math.PI / 180.0) * ShooterConstants.ANGLE_TOLERANCE;

        if (Math.abs(error) > toleranceRads) {
            turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);
        } else {
            turnPower = 0.0;
        }

        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        follower.setTeleOpDrive(-strafe, forward, -turnPower, true);
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}