// Ficheiro: autos/AutoShootThree.java
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.robot.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
@Disabled
@Autonomous(name = "Auto: Vermelho Triangulo pequeno")
public class AutoRedTuff extends CommandOpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        RobotContainer robot = new RobotContainer(hardwareMap, telemetryM, null, null, AllianceEnum.Red);
        DataStorage.alliance = AllianceEnum.Red;

        Command autonomousCommand = robot.getAutonomousRedTuffCommand();
        while (!isStarted()) {
            robot.updateRobotPose(AllianceEnum.Red, RedRearPoses.getPose(PosesNames.StartPose));
        }
        schedule(autonomousCommand);

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetryM.update();

    }
}