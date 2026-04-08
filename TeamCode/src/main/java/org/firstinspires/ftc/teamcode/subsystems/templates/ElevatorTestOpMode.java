package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: Advanced Elevator", group = "Templates 2026")
public class ElevatorTestOpMode extends CommandOpMode {

    private ElevatorSubsystem elevator;
    private GamepadEx driver;
    private TelemetryManager telemetryManager;

    @Override
    public void initialize() {

        elevator = new ElevatorSubsystem(hardwareMap, telemetryManager);

        register(elevator);

        driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> elevator.setTargetState(ElevatorSubsystem.ElevatorState.RETRACTED)));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> elevator.setTargetState(ElevatorSubsystem.ElevatorState.LOW_BASKET)));

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> elevator.setTargetState(ElevatorSubsystem.ElevatorState.HIGH_BASKET)));
    }
}