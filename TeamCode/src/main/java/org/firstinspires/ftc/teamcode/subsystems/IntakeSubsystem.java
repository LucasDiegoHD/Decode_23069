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
        triggerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
    }

    /**
     * Runs the intake motor to collect game pieces.
     */
    public void run() {
        intakeMotor.setPower(1.0);
    }

    /**
     * Reverses the intake motor.
     */
    public void reverse() {
        intakeMotor.setPower(-1.0);
        triggerMotor.setPower(1.0);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
        triggerMotor.setPower(0);
    }

    /**
     * Runs the trigger motor.
     */
    public void runTrigger() {
        triggerMotor.setPower(-1.0);
        intakeMotor.setPower(1.0);
    }

    /**
     * Stops the trigger motor.
     */
    public void stopTrigger() {
        triggerMotor.setPower(0);
    }

}