package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autos.paths.BlueRearPoses;
import org.firstinspires.ftc.teamcode.autos.paths.PosesNames;
import org.firstinspires.ftc.teamcode.autos.paths.RedRearPoses;
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.util.List;

/**
 * Fiação do robô: constrói os subsistemas e concentra o laço de controle contínuo.
 *
 * <p>Substitui o antigo {@code RobotContainer} na parte de construção. Bindings de gamepad e
 * montagem de rotinas de autônomo ficam nos OpModes, não aqui.
 */
public class Robot {

    public final DrivetrainSubsystem drivetrain;
    public final IntakeSubsystem intake;
    public final ShooterSubsystem shooter;
    public final VisionSubsystem vision;
    public final IndexerSubsystem indexer;
    public final LEDSubsystem led;

    /**
     * Se a mira automatica do atirador esta ativa. O operador alterna pelo D-PAD.
     * Lido pelo comando continuo de mira a cada iteracao.
     */
    public boolean shooterAutoAdjust = true;

    private final List<LynxModule> allHubs;
    private long tempoDoUltimoLoop = 0;

    public Robot(HardwareMap hardwareMap, TelemetryManager telemetry) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap, telemetry);
        indexer = new IndexerSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap, indexer);
    }

    /**
     * Laço de controle contínuo do robô inteiro, agendado uma única vez pelo
     * {@link RobotOpMode} como um comando {@code infinite}.
     *
     * <p>A ordem é fixa e explícita, e não herdada da ordem de iteração do escalonador. O bulk
     * cache é limpo aqui, na primeira linha, para que toda leitura de sensor do ciclo veja dados
     * frescos — e apenas aqui, para não limpar duas vezes por iteração.
     */
    public void update() {
        clearBulkCache();

        drivetrain.update();
        vision.update();
        indexer.update();
        shooter.update();
        intake.update();
        led.update();
    }

    /** Invalida o bulk cache de todos os hubs. Chamado no início de {@link #update()}. */
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Se o robô está mirando para atirar.
     *
     * <p>O Ivy não expõe qual comando detém um recurso, então isto lê a flag que o comando de
     * alinhamento mantém. Substitui o antigo
     * {@code drivetrain.getCurrentCommand() instanceof AlignToAprilTagCommand}.
     */
    public boolean isShooting() {
        return AlignToAprilTagCommand.isAligning();
    }

    /**
     * Define a pose inicial do teleop: a pose persistida do autônomo se houver uma válida,
     * senão a pose de início padrão da aliança.
     */
    public void applyTeleOpStartPose(AllianceEnum alliance) {
        Pose savedPose = (DataStorage.actualPose != null) ? DataStorage.actualPose : PoseStorage.loadPose();

        if (savedPose != null && !Double.isNaN(savedPose.getX()) && !Double.isNaN(savedPose.getY())) {
            drivetrain.getFollower().setPose(savedPose);
        } else {
            drivetrain.getFollower().setPose(startPoseFor(alliance));
        }
    }

    /** Pose de início padrão da aliança. */
    public static Pose startPoseFor(AllianceEnum alliance) {
        return (alliance == AllianceEnum.Red)
                ? RedRearPoses.getPose(PosesNames.StartPose)
                : BlueRearPoses.getPose(PosesNames.StartPose);
    }

    /** Fixa a pose de partida do autônomo, antes do play. */
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

    /** Se a Limelight está vendo uma tag e tem fix de pose. */
    public boolean hasLimelightFix() {
        return vision.hasTarget();
    }

    /** Reposiciona pela Limelight e atualiza a odometria uma vez. Usado na espera pré-play. */
    public void updateRobotPose(Pose robotPose) {
        double yaw = robotPose.getHeading();
        robotPose = vision.getRobotPose(yaw).orElse(robotPose);
        drivetrain.getFollower().setPose(robotPose);
        drivetrain.update();
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public void printLoopTime() {
        long tempoAtual = System.currentTimeMillis();
        long tempoDoLoop = tempoAtual - tempoDoUltimoLoop;

        PanelsTelemetry.INSTANCE.getTelemetry().addData("⚡ Loop Time (ms)", tempoDoLoop);

        tempoDoUltimoLoop = tempoAtual;
    }
}
