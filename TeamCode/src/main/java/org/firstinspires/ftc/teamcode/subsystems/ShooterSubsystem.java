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

    /*
        Formula for calculating RPM by distance:
        RPM = 520.71 * distance + 3815.97
     */

    private final DcMotorEx rShooterMotor;
    private final DcMotorEx lShooterMotor;
    private final VoltageSensor voltageSensor;
    private final TelemetryManager telemetry;
    private final Servo hoodServoLeft;
    private final Servo hoodServoRight;

    private final PIDFController controller;

    private double targetRPM = 0.0;
    private double hoodPosition = 0.5;
    private double smoothedRPM = 0.0;
    private final double ALPHA = 0.15; // Coeficiente do filtro (0.1 suave - 0.3 reativo)
    private static final double MAX_RPM_AT_12V = 5250;

    // Local constant for hood increment
    private static final double HOOD_INCREMENT = 0.02;

    /**
     * Constructs a new ShooterSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
    public ShooterSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        rShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.RSHOOTER_MOTOR_NAME);
        lShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.LSHOOTER_MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hoodServoLeft = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_LEFT_NAME);
        hoodServoRight = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_RIGHT_NAME);

        controller = new PIDFController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                0  // kF (usamos o Feedforward caseiro no periodic)
        );

        rShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hoodServoRight.setDirection(Servo.Direction.REVERSE);
        rShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize hood to default position
        hoodServoLeft.setPosition(hoodPosition);
        hoodServoRight.setPosition(hoodPosition);
    }
    /**
     * Sets the target velocity of the shooter in RPM.
     * @param rpm The target RPM.
     */
    public void setTargetVelocity(double rpm) {
        targetRPM = Math.max(0, rpm);
        controller.reset();
    }
    /**
     * Completely stops the shooter.
     */
    public void stop() {
        targetRPM = 0;
        rShooterMotor.setPower(0);
        lShooterMotor.setPower(0);
    }
    /**
     * Increases the angle of the hood.
     */
    public void increaseHood() {
        hoodPosition = Math.min(ShooterConstants.MAXIMUM_HOOD, hoodPosition + HOOD_INCREMENT);
        updateHoodServos();
    }
    /**
     * Decreases the angle of the hood.
     */
    public void decreaseHood() {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, hoodPosition - HOOD_INCREMENT);
        updateHoodServos();
    }

    public void setHoodPosition(double position) {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, Math.min(ShooterConstants.MAXIMUM_HOOD, position));
        updateHoodServos();
    }

    private void updateHoodServos() {
        hoodServoLeft.setPosition(hoodPosition);
        hoodServoRight.setPosition(hoodPosition);
    }

    public boolean getShooterAtTarget() {
        // Usamos o RPM suavizado para evitar que flutuações de ruído enganem o comando de tiro
        return Math.abs(smoothedRPM - targetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
    }

    private double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    @Override
    public void periodic() {
        // Sincroniza coeficientes (útil para tunar via dashboard em tempo real)
        controller.setP(ShooterConstants.kP);
        controller.setD(ShooterConstants.kD);

        double currentRawRPM = getCurrentRPM();

        // FILTRO IIR - Remove o jitter das leituras do encoder
        smoothedRPM = (1 - ALPHA) * smoothedRPM + ALPHA * currentRawRPM;

        double power = 0;

        if (targetRPM > 0) {
            if (smoothedRPM < targetRPM * 0.98) {
                power = 1.0;
            } else {
                // FEEDFORWARD CASEIRO
                double currentVoltage = voltageSensor.getVoltage();

                double ff = (targetRPM / MAX_RPM_AT_12V) * (12.0 / currentVoltage);

                double feedback = controller.calculate(smoothedRPM, targetRPM);

                power = ff + feedback;
            }

            power = Math.max(0, Math.min(1.0, power));

            rShooterMotor.setPower(power);
            lShooterMotor.setPower(power);
        } else {
            rShooterMotor.setPower(0);
            lShooterMotor.setPower(0);
        }

        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Shooter RPM (Filtered)", smoothedRPM);
        telemetry.addData("Shooter Power Output", power);
        telemetry.addData("Battery Voltage", voltageSensor.getVoltage());
        telemetry.addData("Hood Position", hoodPosition);
    }
}