package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.HOOD_INCREMENT;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx rShooterMotor, lShooterMotor;
    private final VoltageSensor voltageSensor;
    private final TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Servo hoodServoLeft, hoodServoRight;
    private final PIDFController controller;

    private double targetRPM = 0.0;
    private double hoodPosition = 0.5; // Base position from distance
    private double currentDynamicHoodPos = 0.5; // Final position sent to hardware
    private double smoothedRPM = 0.0;
    private final double ALPHA = 0.15;
    private static final double MAX_RPM_AT_12V = 5250;

    // Feature toggle
    private boolean isLongShotMode = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        rShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.RSHOOTER_MOTOR_NAME);
        lShooterMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.LSHOOTER_MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hoodServoLeft = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_LEFT_NAME);
        hoodServoRight = hardwareMap.get(Servo.class, ShooterConstants.HOOD_SERVO_RIGHT_NAME);

        controller = new PIDFController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 0);

        rShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hoodServoRight.setDirection(Servo.Direction.REVERSE);
        rShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        updateHoodServos();
    }

    public void setTargetVelocity(double rpm) { targetRPM = Math.max(0, rpm); }
    public void stop() { targetRPM = 0; rShooterMotor.setPower(0); lShooterMotor.setPower(0); }

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
    }

    /** Toggles the compensation feature based on distance */
    public void setLongShotMode(boolean active) { this.isLongShotMode = active; }

    public boolean getShooterAtTarget() {
        if (targetRPM <= 0) return false;
        // Long shots use aggressive 85% tolerance due to hood compensation
        double tolerance = isLongShotMode ? ShooterConstants.CADENCE_TOLERANCE_PERCENT : 0.98;
        return smoothedRPM > (targetRPM * tolerance);
    }

    private double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    private void updateHoodServos() {
        hoodServoLeft.setPosition(currentDynamicHoodPos);
        hoodServoRight.setPosition(currentDynamicHoodPos);
    }

    @Override
    public void periodic() {
        smoothedRPM = (1 - ALPHA) * smoothedRPM + ALPHA * getCurrentRPM();

        // --- SELECTIVE DYNAMIC COMPENSATION ---
        double rpmError = targetRPM - smoothedRPM;
        if (isLongShotMode && targetRPM > 100 && rpmError > 0) {
            // Flick the hood up only on long shots when RPM drops
            double offset = rpmError * ShooterConstants.K_HOOD_COMPENSATION;
            currentDynamicHoodPos = hoodPosition + offset;
        } else {
            currentDynamicHoodPos = hoodPosition;
        }

        currentDynamicHoodPos = Math.max(ShooterConstants.MINIMUM_HOOD, Math.min(ShooterConstants.MAXIMUM_HOOD, currentDynamicHoodPos));
        updateHoodServos();

        if (targetRPM > 0) {
            double currentVoltage = voltageSensor.getVoltage();
            double ff = (targetRPM / MAX_RPM_AT_12V) * (12.0 / currentVoltage);
            double feedback = controller.calculate(smoothedRPM, targetRPM);
            double power = Math.max(0, Math.min(1.0, ff + feedback));
            rShooterMotor.setPower(power);
            lShooterMotor.setPower(power);
        } else {
            stop();
        }

        telemetry.addData("Shooter RPM", getCurrentRPM());
        telemetry.addData("Shooter smoothedRPM", smoothedRPM);
        telemetry.addData("Shooter TargetRPM", targetRPM);
        telemetry.addData("Shooter RPM Error", rpmError);
        telemetry.addData("Shooter Hood Position", hoodPosition);
    }
}