package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem extends SubsystemBase {

    private final DcMotorEx leftLift, rightLift;
    // private final TelemetryManager telemetry;
    private final Telemetry telemetry;
    private final PIDFController controller;

    private double lastPower = 0;
    private int targetPosition = 0;

    public enum ElevatorState {
        RETRACTED(ElevatorConstants.POS_RETRACTED),
        LOW_BASKET(ElevatorConstants.POS_LOW_BASKET),
        HIGH_BASKET(ElevatorConstants.POS_HIGH_BASKET),
        MANUAL(-1);

        public final int targetPosition;
        ElevatorState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }

    private ElevatorState currentState = ElevatorState.RETRACTED;

    public ElevatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftLift = hardwareMap.get(DcMotorEx.class, ElevatorConstants.LEFT_MOTOR_NAME);
        rightLift = hardwareMap.get(DcMotorEx.class, ElevatorConstants.RIGHT_MOTOR_NAME);

        rightLift.setDirection(DcMotor.Direction.REVERSE);
        resetEncoder();

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 0);
    }

    public void setTargetState(ElevatorState state) {
        this.currentState = state;
        this.targetPosition = state.targetPosition;
    }
    public void setUp () {
        leftLift.setPower(1);
        rightLift.setPower(1);
    }
    public void setDown () {
        leftLift.setPower(-1);
        rightLift.setPower(-1);
    }

    public void manualControl(double upTrigger, double downTrigger) {
        double netTrigger = upTrigger - downTrigger;

        if (Math.abs(netTrigger) > 0.05) {
            this.currentState = ElevatorState.MANUAL;

            targetPosition += (int)(netTrigger * ElevatorConstants.MANUAL_SPEED_TICKS);

            if (targetPosition > ElevatorConstants.MAX_POSITION_TICKS) {
                targetPosition = ElevatorConstants.MAX_POSITION_TICKS;
            } else if (targetPosition < ElevatorConstants.MIN_POSITION_TICKS) {
                targetPosition = ElevatorConstants.MIN_POSITION_TICKS;
            }
        }
    }

    public void resetEncoder() {
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
    }

    @Override
    public void periodic() {
        int currentPos = leftLift.getCurrentPosition();
        double pidOutput = controller.calculate(currentPos, targetPosition);

        double effectiveKF = ElevatorConstants.kF;
        if (targetPosition == 0 && currentPos < 300) {
            effectiveKF = ElevatorConstants.kF * 0.5;
        }

        double power = pidOutput + effectiveKF;

        if (currentPos <= ElevatorConstants.MIN_POSITION_TICKS && power < 0) {
            power = 0;
        }

        if (Math.abs(power - lastPower) > 0.001) {
            leftLift.setPower(power);
            rightLift.setPower(power);
            lastPower = power;
        }

        telemetry.addData("Elevator State", currentState.name());
        telemetry.addData("Elevator Pos", currentPos);
        telemetry.addData("Elevator Target", targetPosition);
    }
}