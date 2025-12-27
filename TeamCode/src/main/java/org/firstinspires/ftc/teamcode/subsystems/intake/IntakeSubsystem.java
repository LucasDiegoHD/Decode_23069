package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * The IntakeSubsystem is responsible for controlling the intake mechanism of the robot.
 * It includes a motor for collecting game pieces and a trigger motor.
 */
//@AutoLog
public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final DcMotor triggerMotor;

    /**
     * Constructs a new IntakeSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        triggerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Runs the intake motor to collect game pieces.
     */
    public void run() { intakeMotor.setPower(1.0);
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
    public void runTrigger() { triggerMotor.setPower(-1.0);
    }

    /**
     * Stops the trigger motor.
     */
    public void stopTrigger() { triggerMotor.setPower(0); }

}