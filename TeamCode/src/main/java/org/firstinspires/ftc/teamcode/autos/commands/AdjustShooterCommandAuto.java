package org.firstinspires.ftc.teamcode.autos.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * A command to spin the shooter motors to a specific velocity or stop them.
 * This is an instant command (finishes immediately) that just sets the state of the shooter.
 */
public class AdjustShooterCommandAuto extends CommandBase {

    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;

    public AdjustShooterCommandAuto (ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        double distance = vision.getDirectDistanceToTarget().orElse((double) 0);

        double rpm = ShooterConstants.RPM_N0 + ShooterConstants.RPM_N1 * distance + ShooterConstants.RPM_N2 * Math.pow(distance, 2);

        if (distance > VisionConstants.LONGEST_DISTANCE) {
            rpm = 2790;
        }
        if (distance == 0){
            rpm = 2790;
        }

        shooter.setTargetVelocity(rpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}