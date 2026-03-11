package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autos.paths.RedFrontPoses; // Importe o novo arquivo
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
@Disabled
@Autonomous(name = "Auto: Vermelho Triangulo GRANDE")
public class AutoRedFront extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    private RobotContainer robot;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new RobotContainer(hardwareMap, telemetryM, null, null, AllianceEnum.Red);
        DataStorage.alliance = AllianceEnum.Red;

        // PEGA O COMANDO NOVO DO FRONT
        Command autonomousCommand = robot.getAutonomousRedFrontCommand();

        while (!isStarted()) {
            robot.updateRobotPose(AllianceEnum.Red, RedFrontPoses.getPose(PosesNames.StartPose));
        }
        schedule(autonomousCommand);
    }

    @Override
    public void run() {
        if (robot != null) {
            robot.clearBulkCache();
        }
        CommandScheduler.getInstance().run();
        telemetryM.update();
    }
}