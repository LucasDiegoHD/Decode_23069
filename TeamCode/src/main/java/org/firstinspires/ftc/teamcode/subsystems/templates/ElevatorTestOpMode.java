package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: Simple Elevator Trigger", group = "Templates 2026")
public class ElevatorTestOpMode extends CommandOpMode {

    private ElevatorSubsystem elevator;
    private GamepadEx driver;
    @Override
    public void initialize() {
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        register(elevator);

        driver = new GamepadEx(gamepad1);

        // Right Trigger sobe, Left Trigger desce.
        elevator.setDefaultCommand(new RunCommand(
                () -> elevator.manualControl(
                        driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                        driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                ),
                elevator
        ));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> elevator.setTargetState(ElevatorSubsystem.ElevatorState.RETRACTED)));

        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> elevator.setTargetState(ElevatorSubsystem.ElevatorState.LOW_BASKET)));

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> elevator.setTargetState(ElevatorSubsystem.ElevatorState.HIGH_BASKET)));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> elevator.resetEncoder()));

        if (gamepad1.left_bumper){
            elevator.setUp();
        }

        if (gamepad1.right_bumper){
            elevator.setDown();
        }
    }
}