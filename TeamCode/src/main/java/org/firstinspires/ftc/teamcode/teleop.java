// Ficheiro: teleop.java
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

@TeleOp
public class teleop extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        // Passa o hardwareMap, a telemetria e os gamepads para o RobotContainer
        RobotContainer robot = new RobotContainer(hardwareMap, telemetryM, driverGamepad, operatorGamepad, DataStorage.alliance);


    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        telemetryM.update();
    }
}