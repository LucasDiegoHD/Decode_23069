// Ficheiro: autos/AutoShootThree.java
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;

@Autonomous(name = "Auto: Vermelho Triangulo pequeno")
public class AutoRedRear extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        RobotContainer robot = new RobotContainer(hardwareMap,null, null, AllianceEnum.Red);
        DataStorage.alliance = AllianceEnum.Red;

        // Pega o NOVO comando que criamos para atirar 3
        Command autonomousCommand = robot.getAutonomousRedRearCommand();

        // Agenda o comando para ser executado após o START
        while (!isStarted()) {
            robot.updateRobotPose(RedRearPoses.getPose(PosesNames.StartPose));
        }

        schedule(autonomousCommand);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryM.update();

    }
}