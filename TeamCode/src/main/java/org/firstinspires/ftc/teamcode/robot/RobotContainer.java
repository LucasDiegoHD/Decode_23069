package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.lynx.LynxModule;
import java.util.List;

import org.firstinspires.ftc.teamcode.autos.commands.AutonomousCommands;
import org.firstinspires.ftc.teamcode.autos.commands.AutonomousFrontCommands;
import org.firstinspires.ftc.teamcode.autos.commands.AutonomousTuffCommand;
import org.firstinspires.ftc.teamcode.autos.paths.BlueFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedFrontPoses;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

/**
 * Main container for robot organization.
 * Features Auto-Periodic Limelight Resync and high-level drive scaling.
 */
public class RobotContainer {

    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final IndexerSubsystem indexer;
    private final LEDSubsystem led;
    private boolean isShooterAutoAdjustActive = true;
    private List<LynxModule> allHubs;
    private long tempoDoUltimoLoop = 0;

    public RobotContainer(HardwareMap hardwareMap, TelemetryManager telemetry, GamepadEx driver, GamepadEx operator, AllianceEnum alliance) {
        // Subsystem Initialization
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap,telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap);


        led.setDefaultCommand(new LedCommand(led, indexer));

        if (driver!= null) {
            Pose savedPose = (DataStorage.actualPose != null) ? DataStorage.actualPose : PoseStorage.loadPose();

            if (savedPose != null && !Double.isNaN(savedPose.getX()) && !Double.isNaN(savedPose.getY())) {
                drivetrain.getFollower().setPose(savedPose);
            } else {
                Pose startPose = (alliance == AllianceEnum.Red) ?
                        RedRearPoses.getPose(PosesNames.StartPose) : BlueRearPoses.getPose(PosesNames.StartPose);
                drivetrain.getFollower().setPose(startPose);
            }


            drivetrain.setDefaultCommand(new TeleOpDriveCommand(drivetrain, driver));

            Pose endPose = (alliance == AllianceEnum.Red)?
                    BlueRearPoses.getPose(PosesNames.EndPose) : RedRearPoses.getPose(PosesNames.EndPose);
            Pose innitialPose = (alliance == AllianceEnum.Red)?
                    RedRearPoses.getPose(PosesNames.EndPose) : BlueRearPoses.getPose(PosesNames.EndPose);
            Pose shootPose = (alliance == AllianceEnum.Red)?
                    RedRearPoses.getPose(PosesNames.GoToShoot1) : BlueRearPoses.getPose(PosesNames.GoToShoot1);

            double goalX = (alliance == AllianceEnum.Red)? 130 : 14;
            double goalY = 130;

            Command periodicUpdateLoop = new RepeatCommand(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new ConditionalCommand(
                                    new InstantCommand(),
                                    new UpdatePoseLimelightCommand(drivetrain, vision, innitialPose),
                                    () -> isRobotShooting() || !drivetrain.isRobotStopped()
                            )
                    )
            );


            CommandScheduler.getInstance().schedule(periodicUpdateLoop);

            shooter.setDefaultCommand(
                    new ActiveAimCommand(shooter, vision, drivetrain, goalX, goalY,
                            () -> isShooterAutoAdjustActive
                    )
            );


            new GamepadButton(driver, GamepadKeys.Button.Y)
                    .whileHeld(new AlignToAprilTagCommand(drivetrain, vision, telemetry, operator));

            double targetx = (alliance == AllianceEnum.Red)? 141 : 3;
            double targety = 144;

            new GamepadButton(driver, GamepadKeys.Button.X)
                    .whileHeld(new KinematicAimDriveCommand(drivetrain, driver, targetx, targety));


            new GamepadButton(driver, GamepadKeys.Button.START)
                    .whenPressed(new InstantCommand(() -> {
                        double targetAngle = 90.0;
                        UpdatePoseLimelightCommand.forceHardReset(drivetrain, vision, targetAngle);
                    }));


            new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                    .whileHeld(new InstantCommand(intake::run, intake))
                    .whenReleased(new InstantCommand(intake::stop, intake));

                    new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)
                            .whileHeld(new AutoShootCommand(drivetrain, vision, shooter, intake, indexer, endPose, led, driver));

        }

        if (operator!= null) {
            configureTeleOpBindings(operator, alliance, driver);
        }
    }

    private void configureTeleOpBindings(GamepadEx operator, AllianceEnum alliance, GamepadEx driver) {
        Pose endPose = (alliance == AllianceEnum.Red)?
                BlueRearPoses.getPose(PosesNames.EndPose) : RedRearPoses.getPose(PosesNames.EndPose);

        new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new AutoShootCommand(drivetrain, vision, shooter, intake, indexer, endPose, led, driver));

        new GamepadButton(operator, GamepadKeys.Button. LEFT_BUMPER)
                .whileHeld(new InstantCommand(intake::run, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intake::reverse, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(intake::runTrigger, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    isShooterAutoAdjustActive = false;
                    shooter.stop();
                }));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> isShooterAutoAdjustActive = true));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    shooter.adjustRpmOffset(10);
                    if (operator.gamepad != null) operator.gamepad.rumble(100);
                }));

        new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {
                    shooter.adjustRpmOffset(-10);
                    if (operator.gamepad != null) operator.gamepad.rumble(100);
                }));

        new GamepadButton(operator, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(shooter::resetRpmOffset));

    }
    public void clearBulkCache() {
        if (allHubs != null) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
    public void printLoopTime() {
        long tempoAtual = System.currentTimeMillis();
        long tempoDoLoop = tempoAtual - tempoDoUltimoLoop;

        PanelsTelemetry.INSTANCE.getTelemetry().addData("⚡ Loop Time (ms)", tempoDoLoop);

        tempoDoUltimoLoop = tempoAtual;
    }

    public void updateRobotPose(AllianceEnum alliance, Pose robotPose) {
        double yaw = robotPose.getHeading();
        robotPose = vision.getRobotPose(yaw).orElse(robotPose);
        drivetrain.getFollower().setPose(robotPose);
        drivetrain.periodic();
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public Command getAutonomousBlueRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, BlueRearPoses.asList(), led);
    }

    public Command getAutonomousRedRearCommand() {
        return new AutonomousCommands(drivetrain, shooter, intake, indexer, vision, RedRearPoses.asList(), led);
    }

    public Command getAutonomousBlueFrontCommand() {
        return new AutonomousFrontCommands(drivetrain, shooter, intake, indexer, vision, BlueFrontPoses.asList(), led);
    }

    public Command getAutonomousRedFrontCommand() {
        return new AutonomousFrontCommands(drivetrain, shooter, intake, indexer, vision, RedFrontPoses.asList(), led);
    }

    public Command getAutonomousRedTuffCommand() {
        return new AutonomousTuffCommand(drivetrain, shooter, intake, indexer, vision, RedRearPoses.asList(), led);
    }

    public Command getAutonomousBlueTuffCommand() {
        return new AutonomousTuffCommand(drivetrain, shooter, intake, indexer, vision, BlueRearPoses.asList(), led);
    }
    private boolean isRobotShooting() {
        Command current = drivetrain.getCurrentCommand();
        return current instanceof AlignToAprilTagCommand;
    }
    public void setAutoStartPose(Pose startPose) {
        drivetrain.getFollower().setStartingPose(startPose);
        drivetrain.getFollower().setPose(startPose);         
    }
    /**
     * Tenta relocalizar via Limelight e aplica se válido.
     * Chamado no loop de espera antes do play para garantir
     * pose inicial correta independente do Pinpoint.
     */
    public void tryRelocalizeLimelight() {
        Pose currentPose = drivetrain.getFollower().getPose();
        double heading = currentPose.getHeading();

        vision.getRobotPoseMT2(heading).ifPresent(llPose -> {
            // Só aplica se a pose da Limelight está próxima da pose esperada
            // (evita aceitar leituras ruins de tags distantes)
            double dist = Math.hypot(
                    llPose.getX() - currentPose.getX(),
                    llPose.getY() - currentPose.getY()
            );
            if (dist < 24.0) { // aceita até 24 inches de diferença
                drivetrain.getFollower().setPose(
                        new Pose(llPose.getX(), llPose.getY(), heading)
                );
            }
        });
    }

    /**
     * Retorna true se a Limelight está vendo uma tag e tem fix de pose.
     */
    public boolean hasLimelightFix() {
        return vision.hasTarget();
    }
}