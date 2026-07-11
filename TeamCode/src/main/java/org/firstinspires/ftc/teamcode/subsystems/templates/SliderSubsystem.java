package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SliderSubsystem extends SubsystemBase {
    private DcMotor sliderMotor;
    private PIDController pidController;
    private int targetPosition = 0;
    private double lastPower = 0;

    public SliderSubsystem(HardwareMap hardwareMap) {
        sliderMotor = hardwareMap.get(DcMotor.class, SliderConstants.SLIDER_MOTOR);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidController = new PIDController(SliderConstants.kP, SliderConstants.kI, SliderConstants.kD);
    }
    public void setTargetPosition(int position) {
        targetPosition = position;
    }
    public double getSliderPosition() {
        return sliderMotor.getCurrentPosition();
    }
    public void resetEncoder() {
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void periodic() {
        double currentPosition = getSliderPosition();
        double power = pidController.calculate(currentPosition, targetPosition);

        if (currentPosition <= SliderConstants.MIN_SLIDER_POSITION && power < 0) {
            power = 0;
        }
        if (Math.abs(power - lastPower) > 0.001) {
            sliderMotor.setPower(power);
            lastPower = power;
        }

    }
}
