package org.firstinspires.ftc.teamcode.autos.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.commands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.commands.UpdatePoseLimelightCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.List;

/**
 * A complex autonomous command sequence for a full autonomous period.
 * This {@link SequentialCommandGroup} orchestrates a series of actions including driving,
 * shooting, and intaking to score points during the autonomous phase of an FTC match.
 *
 * <p>The sequence is as follows:</p>
 * <ol>
 *   <li>Set the robot's starting position.</li>
 *   <li>Spin up the shooter to the correct speed (long or short shot).</li>
 *   <li>Drive to the first shooting position.</li>
 *   <li>Shoot three times.</li>
 *   <li>Drive to the first intake line position, wait briefly, and then drive to catch the pixels.</li>
 *   <li>Drive to the second shooting position, spin up the shooter, and shoot three times.</li>
 *   <li>Drive to the second intake line position, wait, and catch pixels.</li>
 *   <li>Drive to the third shooting position, spin up the shooter, and shoot three times.</li>
 *   <li>Drive to the third intake line position, wait, and catch pixels.</li>
 *   <li>Drive to the fourth shooting position, spin up the shooter (opposite distance of the first shot).</li>
 *   <li>Drive back to the first shooting position and shoot a final three times.</li>
 * </ol>
 * <p>
 * This command group requires the {@link DrivetrainSubsystem}, {@link ShooterSubsystem}, and {@link IntakeSubsystem}.
 */
public class AutonomousTuffCommand extends SequentialCommandGroup {


    /**
     * Creates a new autonomous command sequence.
     * This sequence orchestrates the robot's actions during the autonomous period,
     * including driving to specified poses, shooting rings, and intaking rings.
     * The specific path and shooting distances are determined by the provided poses and the 'LongFirst' flag.
     *
     * @param drivetrain The drivetrain subsystem for robot movement.
     * @param shooter    The shooter subsystem for launching rings.
     * @param intake     The intake subsystem for collecting rings.
     * @param poses      A list of {@link Pose} objects defining the robot's path and key locations.
     *                   The order and meaning of these poses are defined by the {@link PosesNames} enum.
     */
    public AutonomousTuffCommand(@NonNull DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, IntakeSubsystem intake, IndexerSubsystem indexer, VisionSubsystem vision, List<Pose> poses, LEDSubsystem ledSubsystem) {

        addCommands(
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.StartPose.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new UpdatePoseLimelightCommand(drivetrain, vision, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitCommand(1500),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine1.ordinal()),
                        poses.get(PosesNames.CatchLine1.ordinal())
                ).withTimeout(4000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitCommand(500),
                new ShootCommandAutonomous(shooter, intake, indexer,3 ).withTimeout(3000),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine5.ordinal())
                ).withTimeout(2000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.CatchLine2.ordinal())).withTimeout(800),
                new WaitCommand(300),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot2.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitCommand(500),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new WaitCommand(300),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine5.ordinal())
                ).withTimeout(2000),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.CatchLine2.ordinal())).withTimeout(800),
                new WaitCommand(900),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitCommand(500),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000),
                new InstantCommand(intake::run),
                new GoToPoseCommand(drivetrain, true,
                        poses.get(PosesNames.GoToLine2.ordinal()),
                        poses.get(PosesNames.CatchLine2.ordinal())
                ).withTimeout(2000),
                new InstantCommand(intake::stop),
                new GoToPoseCommand(drivetrain, poses.get(PosesNames.GoToShoot1.ordinal())),
                new AlignAndAdjustAutoCommand(drivetrain, vision, shooter),
                new WaitCommand(500),
                new ShootCommandAutonomous(shooter, intake, indexer,3).withTimeout(3000)
        );
        addRequirements(drivetrain, shooter, intake);
    }

}
