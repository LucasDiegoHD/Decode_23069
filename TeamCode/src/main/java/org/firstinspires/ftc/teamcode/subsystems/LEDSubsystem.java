package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDSubsystem extends SubsystemBase {
    private final RevBlinkinLedDriver blinkin;

    public LEDSubsystem(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "led_indicator");
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
    }
}