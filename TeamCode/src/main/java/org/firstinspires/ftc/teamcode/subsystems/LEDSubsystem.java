package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LEDSubsystem extends SubsystemBase {
    private final ServoImplEx led;
    private double targetPosition = 0.0;
    public static final double RED = 0.277;
    public static final double GREEN = 0.500;
    public static final double BLUE = 0.611;
    public static final double OFF = 0.0;
    public static final double WHITE = 1.0;
    public static final double ORANGE = 0.333;
    public static final double YELLOW = 0.338;

    public LEDSubsystem(HardwareMap hardwareMap) {
        led = hardwareMap.get(ServoImplEx.class, "led_indicator");

        led.setPwmRange(new PwmControl.PwmRange(500, 2500));

        this.targetPosition = RED;
    }

    public void setPattern(double position) {
        this.targetPosition = position;
    }

    @Override
    public void periodic() {
        led.setPosition(targetPosition);
    }
}