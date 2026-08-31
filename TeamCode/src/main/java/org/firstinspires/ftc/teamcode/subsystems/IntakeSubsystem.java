package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * The IntakeSubsystem is responsible for controlling the intake mechanism of the robot.
 * It includes a motor for collecting game pieces and a trigger motor.
 */
//@AutoLog
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intakeMotor;
    private final DcMotor triggerMotor;
    private final TelemetryManager telemetry;
    private double lastIntakePower = -999.0;
    private double lastTriggerPower = -999.0;

    /**
     * Constructs a new IntakeSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     */
    public IntakeSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
    }

    private void setIntakePower(double targetPower) {
        if (Math.abs(targetPower - lastIntakePower) > 0.01) {
            intakeMotor.setPower(targetPower);
            lastIntakePower = targetPower;
        }
    }

    private void setTriggerPower(double targetPower) {
        if (Math.abs(targetPower - lastTriggerPower) > 0.01) {
            triggerMotor.setPower(targetPower);
            lastTriggerPower = targetPower;
        }
    }

    /**
     * Runs the intake motor to collect game pieces.
     */
    /**
     * Runs the intake motor to collect game pieces.
     */
    public void run() {
        setIntakePower(1.0);
    }

    /**
     * Reverses the intake motor.
     */
    public void reverse() {
        setIntakePower(-1.0);
        setTriggerPower(1.0);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        setIntakePower(0.0);
        setTriggerPower(0.0);
    }

    /**
     * Runs the trigger motor.
     */
    public void runTrigger() {
        setIntakePower(1.0);
        setTriggerPower(-1.0);
    }

    /**
     * Stops the trigger motor.
     */
    public void stopTrigger() {
        setTriggerPower(0.0);
    }

}