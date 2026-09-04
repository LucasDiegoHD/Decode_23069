package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;

/**
 * Macro de tiro do teleop, num botão só.
 *
 * <p>Hoje é apenas o {@link ShootCommand}: os passos de relocalização e de ajuste de capô e
 * velocidade estavam comentados no original e continuam fora — o ajuste contínuo já é feito pelo
 * {@link ActiveAimCommand}, que roda o tempo todo.
 */
public final class AutoShootCommand {

    private AutoShootCommand() {
    }

    public static Command autoShoot(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
                                    ShooterSubsystem shooter, IntakeSubsystem intake,
                                    IndexerSubsystem indexer, Pose fallbackPose, Gamepad driver) {
        return Groups.sequential(
                ShootCommand.shoot(shooter, intake, indexer, driver)
        );
    }
}
