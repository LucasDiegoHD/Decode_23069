package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.BlueFrontPoses; // Importe o novo arquivo
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

@Autonomous(name = "Auto: Azul Triangulo GRANDE")
public class AutoBlueFront extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Inicializa o container
        RobotContainer robot = new RobotContainer(hardwareMap, telemetryM, null, null, AllianceEnum.Blue);
        DataStorage.alliance = AllianceEnum.Blue;

        // PEGA O COMANDO NOVO DO FRONT
        Command autonomousCommand = robot.getAutonomousBlueFrontCommand();

        while (!isStarted()) {
            // Atualiza a pose inicial baseada no Front
            robot.updateRobotPose(AllianceEnum.Blue, BlueFrontPoses.getPose(PosesNames.StartPose));
        }
        schedule(autonomousCommand);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryM.update();
    }
}