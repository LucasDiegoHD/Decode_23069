package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

@TeleOp
public class teleop extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    private RobotContainer robot;
    private long ultimoTempoTelemetria = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        robot = new RobotContainer(hardwareMap, telemetryM, driverGamepad, operatorGamepad, DataStorage.alliance);
    }

    @Override
    public void run() {
        long tempoInicio = System.currentTimeMillis();

        if (robot != null) {
            robot.clearBulkCache();
        }
        long tempoCache = System.currentTimeMillis();

        super.run();

        long tempoComandos = System.currentTimeMillis();

        telemetryM.addData("1. Tempo do Cache (ms)", tempoCache - tempoInicio);
        telemetryM.addData("2. Tempo dos Comandos (ms)", tempoComandos - tempoCache);
        telemetryM.addData("3. Tempo da Telemetria (ms)", ultimoTempoTelemetria);

        long tempoTotal = (tempoComandos - tempoInicio) + ultimoTempoTelemetria;
        telemetryM.addData("⚡ TOTAL LOOP TIME (ms)", tempoTotal);

        long tempoAntesTelemetria = System.currentTimeMillis();
        telemetryM.update();
        ultimoTempoTelemetria = System.currentTimeMillis() - tempoAntesTelemetria;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        CommandScheduler.getInstance().reset();
    }
}