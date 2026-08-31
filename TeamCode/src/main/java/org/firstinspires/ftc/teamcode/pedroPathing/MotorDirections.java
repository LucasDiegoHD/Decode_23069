package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;


@TeleOp(name = "Motor Directions", group = "Teleop Test")
@Disabled
public class MotorDirections extends OpMode {
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        leftFront = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName );
        leftRear = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        List<DcMotorEx> motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }


    }

    @Override
    public void loop() {


        if(gamepad1.a)
            leftFront.setPower(1);
        else
            leftFront.setPower(0);

        if(gamepad1.y)
            leftRear.setPower(1);
        else
            leftRear.setPower(0);

        if(gamepad1.b)
            rightFront.setPower(1);
        else
            rightFront.setPower(0);

        if(gamepad1.x)
            rightRear.setPower(1);
        else
            rightRear.setPower(0);


        telemetryM.addLine("Press A to spin the left front motor at 100% power");
        telemetryM.addLine("Press Y to spin the left rear motor at 100% power");
        telemetryM.addLine("Press B to spin the right front motor at 100% power");
        telemetryM.addLine("Press X to spin the right rear motor at 100% power");

        telemetryM.update();
    }
}

