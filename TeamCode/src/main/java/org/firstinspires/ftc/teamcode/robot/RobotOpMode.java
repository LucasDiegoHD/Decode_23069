package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Base iterativa de todos os OpModes de competição.
 *
 * <p>Substitui o {@code CommandOpMode} da FTCLib. O ciclo é o do {@link OpMode} do SDK
 * ({@code init} → {@code init_loop} → {@code start} → {@code loop} → {@code stop}), sem laço
 * bloqueante: a configuração pré-play é feita no {@link #init_loop()}.
 *
 * <p>O {@code Scheduler} do Ivy é estático, então o estado sobrevive entre execuções de OpMode.
 * Por isso o reset acontece no {@link #init()} <em>e</em> no {@link #stop()} — sem isso, comandos
 * de uma execução vazam para a seguinte.
 *
 * <p>Todo o controle contínuo do robô vive num único comando {@code infinite} que chama
 * {@link Robot#update()}. É lá que o bulk cache dos hubs é limpo, antes de qualquer leitura de
 * sensor do ciclo; não limpe o cache aqui também.
 */
public abstract class RobotOpMode extends OpMode {

    @IgnoreConfigurable
    protected static TelemetryManager telemetryM;

    protected Robot robot;

    /** O comando contínuo do robô, guardado para permitir checagens de estado. */
    private Command robotUpdateCommand;

    @Override
    public void init() {
        Scheduler.reset();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap, telemetryM);

        robotUpdateCommand = Commands.infinite(robot::update);
        Scheduler.schedule(robotUpdateCommand);
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
        telemetryM.update();
    }

    @Override
    public void loop() {
        Scheduler.execute();
        telemetryM.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
    }
}
