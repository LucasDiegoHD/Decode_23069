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

    // --- VARIÁVEIS DE CALIBRAÇÃO (MIRA ANTI-DEFESA) ---
    // Velocidade teórica do artefato com sushi roller (ajuste na quadra se a bola errar pros lados)
    private static final double ARTIFACT_VELOCITY_INCHES_PER_SEC = 400.0;

    // Latência do servo (tempo entre o código mandar atirar e o artefato sair da roda)
    private static final double SYSTEM_LATENCY_SECONDS = 0.20;

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
    }

    @Override
    public void execute() {
        // Lemos apenas o analógico esquerdo (Translação: Frente/Trás e Strafe)
        double forward = -driver.getLeftX();
        double strafe = -driver.getLeftY();

        // Puxamos a odometria e a velocidade real do robô (super rápido graças ao Bulk Caching!)
        Pose pose = follower.getPose();
        Vector velocity = follower.getVelocity();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double heading = pose.getHeading();

        // Matemática Preditiva
        double distanceToTarget = Math.hypot(targetX - robotX, targetY - robotY);
        double timeOfFlight = distanceToTarget / ARTIFACT_VELOCITY_INCHES_PER_SEC;
        double totalPredictionTime = timeOfFlight + SYSTEM_LATENCY_SECONDS;

        // O Alvo Virtual que compensa a fuga lateral do robô
        double virtualX = targetX - (velocity.getXComponent() * totalPredictionTime);
        double virtualY = targetY - (velocity.getYComponent() * totalPredictionTime);

        // Calcula o ângulo para o alvo virtual e aciona o PID
        double desiredAngle = Math.atan2(virtualY - robotY, virtualX - robotX);
        double error = angleDifference(desiredAngle, heading);

        turnController.setPIDF(ShooterConstants.ANGLE_KP, ShooterConstants.ANGLE_KI, ShooterConstants.ANGLE_KD, ShooterConstants.ANGLE_KF);
        double turnPower = turnController.calculate(error);
        turnPower += Math.copySign(ShooterConstants.ANGLE_KF, turnPower);

        // Trava o limite de força do motor
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        // Piloto domina a translação, Computador domina a rotação
        follower.setTeleOpDrive(forward, strafe, -turnPower, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Ao soltar o botão, zera os motores para devolver suavemente ao TeleOp normal
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}