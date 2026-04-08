package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ElevatorSubsystem extends SubsystemBase {

    private final DcMotorEx leftLift, rightLift;
    private final VoltageSensor voltageSensor;

    private final TelemetryManager telemetry;
    private final ProfiledPIDController controller;

    private double lastPower = 0;
    private boolean hasGamePiece = false;

    public enum ElevatorState {
        RETRACTED(ElevatorConstants.POS_RETRACTED),
        LOW_BASKET(ElevatorConstants.POS_LOW_BASKET),
        HIGH_BASKET(ElevatorConstants.POS_HIGH_BASKET);

        public final int targetPosition;
        ElevatorState(int targetPosition) {
            this.targetPosition = targetPosition;
        }
    }

    private ElevatorState currentState = ElevatorState.RETRACTED;

    public ElevatorSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        leftLift = hardwareMap.get(DcMotorEx.class, ElevatorConstants.LEFT_MOTOR_NAME);
        rightLift = hardwareMap.get(DcMotorEx.class, ElevatorConstants.RIGHT_MOTOR_NAME);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftLift.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY,
                ElevatorConstants.MAX_ACCELERATION
        );

        controller = new ProfiledPIDController(
                ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, constraints
        );
    }

    public void setTargetState(ElevatorState state) {
        this.currentState = state;
        controller.setGoal(state.targetPosition);
    }
    public void setHasGamePiece(boolean hasPiece) {
        this.hasGamePiece = hasPiece;
    }

    public int getCurrentPosition() {
        return leftLift.getCurrentPosition();
    }

    public void stop() {
        leftLift.setPower(0);
        rightLift.setPower(0);
        lastPower = 0;
    }

    @Override
    public void periodic() {
        int currentPos = getCurrentPosition();

        double pidOutput = controller.calculate(currentPos);
        TrapezoidProfile.State setpoint = controller.getSetpoint();

        double dynamicKG = ElevatorConstants.kG;
        if (hasGamePiece) {
            dynamicKG += ElevatorConstants.kG_EXTRA_PAYLOAD;
        }

        double ffOutput = (ElevatorConstants.kS * Math.signum(setpoint.velocity))
                + dynamicKG
                + (ElevatorConstants.kV * setpoint.velocity);

        double rawPower = pidOutput + ffOutput;

        double voltage = voltageSensor.getVoltage();
        double voltageMultiplier = 12.0 / voltage;
        double power = rawPower * voltageMultiplier;

        if (currentPos <= ElevatorConstants.MIN_POSITION_TICKS && power < 0) {
            power = 0;
        } else if (currentPos >= ElevatorConstants.MAX_POSITION_TICKS && power > 0) {
            power = dynamicKG * voltageMultiplier;
        }

        // Otimização I2C
        if (Math.abs(power - lastPower) > 0.0005) {
            leftLift.setPower(power);
            rightLift.setPower(power);
            lastPower = power;
        }

        telemetry.addData("Elevator State", currentState.name());
        telemetry.addData("Elevator Pos", currentPos);
        telemetry.addData("Elevator Target", controller.getGoal().position);
        telemetry.addData("Payload Active", hasGamePiece);
        telemetry.addData("Voltage Multiplier", voltageMultiplier);
    }
}