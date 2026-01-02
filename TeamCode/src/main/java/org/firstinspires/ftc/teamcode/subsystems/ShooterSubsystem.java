package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx rShooterMotor, lShooterMotor;
    private final VoltageSensor voltageSensor;
    private final TelemetryManager telemetry;
    private final Servo hoodServoLeft, hoodServoRight;
    private final PIDFController controller;

    private double targetRPM = 0.0;
    private double hoodPosition = 0.50;
    private double currentDynamicHoodPos = 0.50;
    private double lastPower = 0;
    private boolean isLongShotMode = false;

    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        rShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.RSHOOTER_MOTOR_NAME);
        lShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.LSHOOTER_MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hoodServoLeft = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_LEFT_NAME);
        hoodServoRight = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_RIGHT_NAME);

        controller = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 0);

        rShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Elite: Usamos RUN_WITHOUT_ENCODER para controle manual de alta frequência
        rShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Elite: Flywheels devem rodar livre (FLOAT) ao parar para não forçar as engrenagens
        rShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hoodServoRight.setDirection(Servo.Direction.REVERSE);
        rShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        updateHoodServos();
    }

    public void setTargetVelocity(double rpm) { targetRPM = Math.max(0, rpm); }

    public void stop() {
        targetRPM = 0;
        controller.reset(); // LIMPA O ERRO ACUMULADO (Evita o giro fantasma)
        rShooterMotor.setPower(0);
        lShooterMotor.setPower(0);
        lastPower = 0;
    }

    public void increaseHood() {
        hoodPosition = Math.min(ShooterConstants.MAXIMUM_HOOD, hoodPosition + ShooterConstants.HOOD_INCREMENT);
    }

    public void decreaseHood() {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, hoodPosition - ShooterConstants.HOOD_INCREMENT);
    }

    public void setHoodPosition(double position) {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, Math.min(ShooterConstants.MAXIMUM_HOOD, position));
    }

    public void setLongShotMode(boolean active) { this.isLongShotMode = active; }

    public boolean isPredictiveReady() {
        return targetRPM > 50 && getCurrentRPM() > (targetRPM * ShooterConstants.CADENCE_TOLERANCE_PERCENT);
    }

    /** Retorna true se estiver na janela final de disparo */
    public boolean getShooterAtTarget() {
        if (targetRPM <= 50) return false;
        return getCurrentRPM() > (targetRPM * ShooterConstants.CADENCE_TOLERANCE_PERCENT);
    }

    public double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    private void updateHoodServos() {
        hoodServoLeft.setPosition(currentDynamicHoodPos);
        hoodServoRight.setPosition(currentDynamicHoodPos);
    }

    @Override
    public void periodic() {
        double currentRPM = getCurrentRPM();
        double rpmError = targetRPM - currentRPM;

        // --- TRAVA DE SEGURANÇA ELITE ---
        // Só aplica potência se o alvo for significativo (> 50 RPM)
        if (targetRPM > 50) {
            double v = voltageSensor.getVoltage();

            // Feedforward: kS (vencer inércia) + kV (velocidade)
            // O kS aqui é multiplicado pelo sinal do target, garantindo 0 quando parado.
            double feedforward = (ShooterConstants.kS * Math.signum(targetRPM) + ShooterConstants.kV * targetRPM) * (12.0 / v);

            double feedback = controller.calculate(currentRPM, targetRPM);

            double power = Math.max(0, Math.min(1.0, feedforward + feedback));

            if (Math.abs(power - lastPower) > 0.0005) {
                rShooterMotor.setPower(power);
                lShooterMotor.setPower(power);
                lastPower = power;
            }
        } else {
            stop();
        }

        if (isLongShotMode && targetRPM > 100 && rpmError > 0) {
            double offset = rpmError * ShooterConstants.K_HOOD_COMPENSATION;
            currentDynamicHoodPos = hoodPosition + offset;
        } else {
            currentDynamicHoodPos = hoodPosition;
        }

        currentDynamicHoodPos = Math.max(ShooterConstants.MINIMUM_HOOD,
                Math.min(ShooterConstants.MAXIMUM_HOOD, currentDynamicHoodPos));
        updateHoodServos();

        telemetry.addData("Shooter RPM Real", currentRPM);
        telemetry.addData("Shooter Target", targetRPM);
        telemetry.addData("Shooter Power Sent", lastPower);
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Hood Position", hoodPosition);
    }
}