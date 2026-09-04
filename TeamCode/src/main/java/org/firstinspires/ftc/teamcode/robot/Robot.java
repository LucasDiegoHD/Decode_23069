package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.templates.ShooterSubsystem;

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

    private final List<LynxModule> allHubs;

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
}
