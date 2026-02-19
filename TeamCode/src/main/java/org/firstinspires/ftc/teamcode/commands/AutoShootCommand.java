package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * This command automates the entire shooting sequence with a single button press.
 * It aligns to the target, adjusts the shooter, waits for it to be ready, and then launches the note.
 */
public class AutoShootCommand extends SequentialCommandGroup {

    /**
     * Creates a new AutoShootCommand.
     *
     * @param drivetrain The drivetrain subsystem for alignment.
     * @param vision     The vision subsystem for target detection.
     * @param shooter    The shooter subsystem for launching the note.
     * @param intake     The intake subsystem for feeding the note.
     */
    public AutoShootCommand(DrivetrainSubsystem drivetrain, VisionSubsystem vision, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, Pose fallbackPose, LEDSubsystem ledSubsystem) {
        addCommands(

                //new UpdatePoseLimelightCommand(drivetrain, vision, fallbackPose),
                //new AdjustHoodCommand(shooter, vision),

                //new AdjustShooterCommand(shooter, vision),

                new ShootCommand(shooter, intake, indexer)
        );
    }
}
