package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceEnum;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.Polygon2d;

public class TeleOpDriveCommandZoneRepulsion extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final GamepadEx driverGamepad;
    private final AllianceEnum alliance;

    private final Polygon2d exclusionZone;
    private final double repulsionStrength;

    /**
     * @param drivetrain       Subsistema de tração
     * @param driverGamepad    Gamepad do piloto
     * @param exclusionZone    O objeto Polygon2d definindo a área proibida
     * @param repulsionStrength Força de repulsão (0.0 a 1.0)
     */
    public TeleOpDriveCommandZoneRepulsion(DrivetrainSubsystem drivetrain, GamepadEx driverGamepad,
                                              Polygon2d exclusionZone, double repulsionStrength) {
        this.drivetrain = drivetrain;
        this.driverGamepad = driverGamepad;
        this.alliance = DataStorage.alliance;
        this.exclusionZone = exclusionZone;
        this.repulsionStrength = repulsionStrength;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.getFollower().startTeleopDrive();
    }

    @Override
    public void execute() {
        Pose currentPose = drivetrain.getFollower().getPose();
        double heading = currentPose.getHeading();

        // Conversão de Pose (Pedro) para Translation2d (FTCLib) para usar na lógica do polígono
        Translation2d robotPos = new Translation2d(currentPose.getX(), currentPose.getY());

        // 1. Inputs do Joystick
        double joyY = driverGamepad.getLeftX();
        double joyX = driverGamepad.getLeftY();
        double joyTurn = -driverGamepad.getRightX();

        // 2. Field Centric
        double xField = joyX * Math.cos(heading) - joyY * Math.sin(heading);
        double yField = joyX * Math.sin(heading) + joyY * Math.cos(heading);

        if (alliance == AllianceEnum.Blue) {
            xField = -xField;
            yField = -yField;
        }

        // 3. Lógica de Repulsão Poligonal
        double repulsionX = 0;
        double repulsionY = 0;

        if (exclusionZone.containsPoint(robotPos)) {
            // Se estamos dentro, encontramos o ponto mais próximo na borda para sair o mais rápido possível
            Translation2d closestPoint = getClosestPointOnPerimeter(robotPos, exclusionZone.getVertices());

            // Vetor: Do ponto da borda -> Para o Robô (Empurra para fora)
            double deltaX = robotPos.getX() - closestPoint.getX();
            double deltaY = robotPos.getY() - closestPoint.getY();

            double distance = Math.hypot(deltaX, deltaY);

            // Normaliza e aplica força. Se distance for muito pequena (ex: 0), definimos um vetor padrão
            if (distance > 1e-6) {
                repulsionX = (deltaX / distance) * repulsionStrength;
                repulsionY = (deltaY / distance) * repulsionStrength;
            } else {
                // Caso extremo: robô exatamente em cima da linha, empurra arbitrariamente
                repulsionX = repulsionStrength;
            }
        }

        // 4. Soma e Normalização
        double finalX = xField + repulsionX;
        double finalY = yField + repulsionY;

        double magnitude = Math.hypot(finalX, finalY);
        if (magnitude > 1.0) {
            finalX /= magnitude;
            finalY /= magnitude;
        }

        // 5. Envia ao Drive
        drivetrain.getFollower().setTeleOpDrive(
                finalX,
                -finalY,
                joyTurn,
                true
        );

        // Telemetria
        PanelsTelemetry.INSTANCE.getTelemetry().addData("In Polygon", (Math.abs(repulsionX) > 0));
    }

    /**
     * Algoritmo auxiliar para encontrar o ponto mais próximo no perímetro do polígono.
     * Projeta o ponto do robô em cada segmento de linha do polígono e encontra a menor distância.
     */
    private Translation2d getClosestPointOnPerimeter(Translation2d point, Translation2d[] vertices) {
        double minDistanceSq = Double.MAX_VALUE;
        Translation2d closest = vertices[0];

        for (int i = 0; i < vertices.length; i++) {
            Translation2d p1 = vertices[i];
            Translation2d p2 = vertices[(i + 1) % vertices.length]; // Conecta o último ao primeiro

            Translation2d projection = getClosestPointOnSegment(p1, p2, point);

            double distSq = Math.pow(point.getX() - projection.getX(), 2) +
                    Math.pow(point.getY() - projection.getY(), 2);

            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                closest = projection;
            }
        }
        return closest;
    }

    /**
     * Projeta o ponto P no segmento de linha AB e garante que a projeção fique dentro do segmento.
     */
    private Translation2d getClosestPointOnSegment(Translation2d a, Translation2d b, Translation2d p) {
        double x1 = a.getX(), y1 = a.getY();
        double x2 = b.getX(), y2 = b.getY();
        double px = p.getX(), py = p.getY();

        double dx = x2 - x1;
        double dy = y2 - y1;

        if (dx == 0 && dy == 0) return a; // Segmento é um ponto

        // Projeção escalar t
        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);

        // Clampa t entre 0 e 1 para garantir que o ponto esteja no segmento
        t = Math.max(0, Math.min(1, t));

        return new Translation2d(x1 + t * dx, y1 + t * dy);
    }
}