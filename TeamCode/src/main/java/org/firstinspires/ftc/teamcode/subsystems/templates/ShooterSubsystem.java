package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

/**
 * Dual Flywheel Shooter with Dynamic Variable Hood Subsystem Template
 *
 * Overview:
 * Controls a high-velocity dual-flywheel launching mechanism paired with dual synchronized
 * servos that adjust the hood exit angle. The mechanism is capable of variable-distance scoring
 * using closed-loop velocity PIDF with battery voltage compensation and distance-based polynomial trajectory mapping.
 *
 * Hardware Configuration:
 * - Right Flywheel Motor (DcMotorEx): "rightShooterMotor"
 * - Left Flywheel Motor (DcMotorEx):  "leftShooterMotor"
 * - Left Hood Servo:                  "Hood_ServoL"
 * - Right Hood Servo:                 "Hood_ServoR" (Reversed)
 * - Voltage Sensor:                   Acquired via hardwareMap.voltageSensor
 *
 * Control Strategy and Features:
 * - Velocity Control: Combines static and velocity feedforward (kS, kV) scaled by
 *   (12.0 / battery_voltage) with FTCLib PIDFController feedback.
 * - Shot Boost: Provides a momentary feedforward impulse (kF_SHOT_BOOST) during
 *   gate trigger actuation to prevent RPM drop upon piece contact.
 * - Dynamic Hood Trimming: Fine-tunes servo positions in real time based on instantaneous
 *   flywheel velocity errors to maintain consistent ballistic trajectories.
 * - Distance-Scaled Tolerance: Dynamically relaxes or tightens RPM readiness tolerances
 *   based on target distance from the goal.
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
public class ShooterSubsystem {

    private final DcMotorEx rShooterMotor, lShooterMotor;
    private final VoltageSensor voltageSensor;
    private final TelemetryManager telemetry;
    private final Servo hoodServoLeft, hoodServoRight;
    private final PIDFController controller;
    private double targetRPM = 0.0;
    private double hoodPosition = 0.7;
    private double currentDynamicHoodPos = 0.7;
    private double lastPower = 0;
    private boolean isLongShotMode = false;
    private final ElapsedTime shotBoostTimer = new ElapsedTime();
    private boolean isBoostActive = false;
    private double currentDistance = 99.0;
    private double liveRpmOffset = 0.0;

    /**
     * Constructs a new ShooterSubsystem, initializes motors, servos, encoders, and PIDF controllers.
     *
     * @param hardwareMap Robot hardware map for device retrieval.
     * @param telemetry   Telemetry manager for diagnostic logging.
     */
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

        rShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lShooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        lShooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hoodServoRight.setDirection(Servo.Direction.REVERSE);
        rShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        updateHoodServos();
    }

    /**
     * Sets the closed-loop target velocity for the shooter flywheels in RPM.
     *
     * @param rpm Target flywheel rotational velocity in revolutions per minute.
     */
    public void setTargetVelocity(double rpm) {
        targetRPM = Math.max(0, rpm);
    }

    /**
     * Stops the flywheel motors and resets the PIDF controller.
     */
    public void stop() {
        targetRPM = 0;
        controller.reset();
        rShooterMotor.setPower(0);
        lShooterMotor.setPower(0);
        lastPower = 0;
        isBoostActive = false;
    }

    /**
     * Increments the target hood servo position by HOOD_INCREMENT.
     */
    public void increaseHood() {
        hoodPosition = Math.min(ShooterConstants.MAXIMUM_HOOD, hoodPosition + ShooterConstants.HOOD_INCREMENT);
    }

    /**
     * Decrements the target hood servo position by HOOD_INCREMENT.
     */
    public void decreaseHood() {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, hoodPosition - ShooterConstants.HOOD_INCREMENT);
    }

    /**
     * Sets the normalized target position for the hood angle servos.
     *
     * @param position Desired position (clamped between MINIMUM_HOOD and MAXIMUM_HOOD).
     */
    public void setHoodPosition(double position) {
        hoodPosition = Math.max(ShooterConstants.MINIMUM_HOOD, Math.min(ShooterConstants.MAXIMUM_HOOD, position));
    }

    /**
     * Sets whether long-shot compensation mode is active.
     *
     * @param active True for long-distance shot compensation; false otherwise.
     */
    public void setLongShotMode(boolean active) {
        this.isLongShotMode = active;
    }

    /**
     * Updates the current estimated physical distance to the target goal in meters.
     *
     * @param distanceMeters Distance in meters.
     */
    public void setCurrentDistance(double distanceMeters) {
        this.currentDistance = distanceMeters;
    }

    /**
     * Checks whether the current flywheel RPM is within the distance-scaled tolerance of the target RPM.
     *
     * @return True if flywheel velocity meets firing criteria; false otherwise.
     */
    public boolean getShooterAtTarget() {
        if (targetRPM <= 50) return false;
        double tolerance = calcularTolerancia(currentDistance);
        return getCurrentRPM() > (targetRPM * tolerance);
    }

    /**
     * Calculates the required RPM tolerance percentage as a function of target distance.
     *
     * @param distanceMeters Target distance in meters.
     * @return Required ratio of target RPM (e.g. 0.80 to 0.94).
     */
    private double calcularTolerancia(double distanceMeters) {
        double minTolerance = 0.80;
        double maxTolerance = ShooterConstants.CADENCE_TOLERANCE_PERCENT;

        if (distanceMeters >= VisionConstants.LONGEST_DISTANCE) return maxTolerance;
        if (distanceMeters <= VisionConstants.LONGEST_DISTANCE * 0.7) return minTolerance;

        double t = (distanceMeters - VisionConstants.LONGEST_DISTANCE * 0.7) /
                (VisionConstants.LONGEST_DISTANCE * 0.3);
        return minTolerance + t * (maxTolerance - minTolerance);
    }

    /**
     * Checks whether the flywheel is at steady state (>96% of target RPM).
     *
     * @return True if flywheel speed is ready.
     */
    public boolean isReady() {
        if (targetRPM <= 50) return false;
        return getCurrentRPM() > (targetRPM * 0.96);
    }

    /**
     * Computes the average measured RPM of both flywheel motors from encoder tick rates.
     *
     * @return Current rotational speed in RPM.
     */
    public double getCurrentRPM() {
        double ticksPerSecond = (rShooterMotor.getVelocity() + lShooterMotor.getVelocity()) / 2.0;
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    /**
     * Transmits the current dynamic hood position to both left and right servos.
     */
    private void updateHoodServos() {
        hoodServoLeft.setPosition(currentDynamicHoodPos);
        hoodServoRight.setPosition(currentDynamicHoodPos);
    }

    /**
     * Triggers the shot boost feedforward timer prior to a piece being fed into the flywheels.
     */
    public void anticipateShot() {
        shotBoostTimer.reset();
        isBoostActive = true;
    }

    /**
     * Adjusts the live driver-trimmed RPM offset.
     *
     * @param delta RPM adjustment increment/decrement.
     */
    public void adjustRpmOffset(double delta) {
        liveRpmOffset += delta;
    }

    /**
     * Returns the current live driver RPM offset trim.
     *
     * @return RPM offset.
     */
    public double getLiveRpmOffset() {
        return liveRpmOffset;
    }

    /**
     * Resets the live RPM offset trim back to 0.
     */
    public void resetRpmOffset() {
        liveRpmOffset = 0.0;
    }

    /**
     * Periodic closed-loop control loop. Calculates feedforward, PIDF feedback, voltage compensation,
     * dynamic hood angle trimming, and publishes telemetry.
     */
    public void update() {
        double currentRPM = getCurrentRPM();
        double rpmError = targetRPM - currentRPM;

        if (targetRPM > 50) {
            double v = voltageSensor.getVoltage();
            double voltageComp = 12.0 / v;

            double feedforward = (ShooterConstants.kS * Math.signum(targetRPM) + ShooterConstants.kV * targetRPM) * voltageComp;
            double shotBoost = 0.0;
            if (isBoostActive) {
                if (shotBoostTimer.milliseconds() < ShooterConstants.SHOT_BOOST_DURATION) {
                    shotBoost = ShooterConstants.kF_SHOT_BOOST * voltageComp;
                } else {
                    isBoostActive = false;
                }
            }

            double power;
            if (rpmError < -150) {
                controller.reset();
                power = 0.0;
            } else {
                double feedback = controller.calculate(currentRPM, targetRPM);
                power = Math.max(0, Math.min(1.0, feedforward + feedback + shotBoost));
            }

            if (Math.abs(power - lastPower) > 0.0005) {
                rShooterMotor.setPower(power);
                lShooterMotor.setPower(power);
                lastPower = power;
            }
        } else {
            stop();
        }

        // Dynamic Hood Trimming based on real-time RPM error
        if (isLongShotMode && targetRPM > 2000 && rpmError > 0) {
            double offset = rpmError * ShooterConstants.K_HOOD_COMPENSATION;
            currentDynamicHoodPos = hoodPosition + offset;
        } else if (!isLongShotMode && targetRPM > 2000 && rpmError < -100) {
            double offset = Math.abs(rpmError) * ShooterConstants.K_ANTI_OVERSHOOT;
            currentDynamicHoodPos = hoodPosition - offset;
        } else {
            currentDynamicHoodPos = hoodPosition;
        }

        currentDynamicHoodPos = Math.max(ShooterConstants.MINIMUM_HOOD,
                Math.min(ShooterConstants.MAXIMUM_HOOD, currentDynamicHoodPos));
        updateHoodServos();

        if (DataStorage.DEBUG_MODE) {
            telemetry.addData("Shooter RPM Real", currentRPM);
            telemetry.addData("Shooter Target", targetRPM);
            telemetry.addData("Shooter Power Sent", lastPower);
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.addData("Hood Position", hoodPosition);
        }
    }
}