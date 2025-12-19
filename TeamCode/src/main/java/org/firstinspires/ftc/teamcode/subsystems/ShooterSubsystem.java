package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


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
                ShooterConstants.kF
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

    public void setMax(){
        rShooterMotor.setPower(1.0);
        lShooterMotor.setPower(1.0);
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
        hoodServoLeft.setPosition(hoodPosition);
        hoodServoRight.setPosition(hoodPosition);

    }

    /**
     * Decreases the angle of the hood.
     */
    public void decreaseHood() {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, hoodPosition - HOOD_INCREMENT);
        hoodServoLeft.setPosition(hoodPosition);
        hoodServoRight.setPosition(hoodPosition);

    }

    public void setHoodPosition(double position) {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, Math.min(ShooterConstants.MAXIMUM_HOOD, position));
        hoodServoLeft.setPosition(hoodPosition);
        hoodServoRight.setPosition(hoodPosition);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Checks if the shooter is at its target velocity.
     * @return True if the current RPM is within the tolerance of the target RPM, false otherwise.
     */
    public boolean getShooterAtTarget() {
        return Math.abs(getCurrentRPM() - targetRPM) < ShooterConstants.VELOCITY_TOLERANCE;
    }

    /**
     * Gets the current average RPM of the shooter motors.
     * @return The current RPM.
     */
    private double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    /**
     * Converts RPM to ticks per second.
     * @param rpm The RPM to convert.
     * @return The equivalent ticks per second.
     */
    protected int RPMToTicks(double rpm) {
        return (int) ((rpm / 60.0) * ShooterConstants.TICKS_PER_REV);
    }

    /**
     * This method is called periodically to update the subsystem's state and telemetry.
     */
    @Override
    public void periodic() {

        controller.setPIDF(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, ShooterConstants.kF);

        double currentRPM = getCurrentRPM();
        double power = 0;

        if (targetRPM > 0) {

            double calculatedPower = controller.calculate(currentRPM, targetRPM);

            // Even if overshoot is extreme (calculatedPower < 0), never apply negative power (reverse)
            // to a spinning flywheel. Cutting power to 0 allows friction to slow it down safely.
            power = Math.max(calculatedPower, 0.0);

            rShooterMotor.setPower(power);
            lShooterMotor.setPower(power);
        } else {
            rShooterMotor.setPower(0);
            lShooterMotor.setPower(0);
        }

        telemetry.addData("Shooter Target RPM", targetRPM);
        telemetry.addData("Shooter Current RPM", currentRPM);
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Shooter Error", targetRPM - currentRPM);
        telemetry.addData("Hood Position", hoodPosition);
    }
}