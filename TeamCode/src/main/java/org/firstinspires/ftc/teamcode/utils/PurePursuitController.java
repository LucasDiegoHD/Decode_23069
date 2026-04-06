package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import java.util.ArrayList;

public class PurePursuitController {

    private PIDController forwardPID;
    private PIDController strafePID;
    private PIDController headingPID;

    private SlewRateLimiter fwdLimiter;
    private SlewRateLimiter strLimiter;
    private SlewRateLimiter turnLimiter;

    private ArrayList<Waypoint> path = new ArrayList<>();
    private int lastFoundIndex = 0;

    public PurePursuitController(PIDController fwd, PIDController str, PIDController head) {
        this.forwardPID = fwd;
        this.strafePID = str;
        this.headingPID = head;

        this.fwdLimiter = new SlewRateLimiter(PurePursuitConstants.ACCEL_FWD);
        this.strLimiter = new SlewRateLimiter(PurePursuitConstants.ACCEL_STR);
        this.turnLimiter = new SlewRateLimiter(PurePursuitConstants.ACCEL_TURN);
    }

    public void setPath(ArrayList<Waypoint> newPath) {
        this.path = newPath;
        this.lastFoundIndex = 0;
        fwdLimiter.reset(0);
        strLimiter.reset(0);
        turnLimiter.reset(0);
    }

    public double[] update(double robotX, double robotY, double robotHeading) {

        if (path == null || path.isEmpty()) return new double[]{0, 0, 0};

        Waypoint targetPoint = getLookaheadPoint(robotX, robotY);
        Waypoint finalPoint = path.get(path.size() - 1);

        if (targetPoint == null) targetPoint = finalPoint;

        // 1. Distância para o DESTINO FINAL (Usado para frear o robô no fim)
        double diffXFinal = finalPoint.x - robotX;
        double diffYFinal = finalPoint.y - robotY;
        double distToFinal = Math.hypot(diffXFinal, diffYFinal);

        // Chegou? Desliga os motores.
        if (lastFoundIndex >= path.size() - 2 && distToFinal < PurePursuitConstants.END_TOLERANCE) {
            return new double[]{0, 0, 0};
        }

        // 2. VETOR DIRECIONAL (A "Bússola" apontando para a cenoura)
        double diffXTarget = targetPoint.x - robotX;
        double diffYTarget = targetPoint.y - robotY;
        double distToTarget = Math.hypot(diffXTarget, diffYTarget);

        // Evita divisão por zero
        if (distToTarget < 0.001) distToTarget = 0.001;

        // Normaliza o vetor (Força = 1.0)
        double dirX = diffXTarget / distToTarget;
        double dirY = diffYTarget / distToTarget;

        // 3. VELOCIDADE GLOBAL (Usa o PID apenas como um acelerador baseado na distância final)
        double baseSpeed = forwardPID.calculate(-distToFinal, 0);

        // Aplica a velocidade no vetor direcional
        double velX = dirX * baseSpeed;
        double velY = dirY * baseSpeed;

        // 4. Rotação Field-Centric para Robot-Centric
        double sin = Math.sin(-robotHeading);
        double cos = Math.cos(-robotHeading);

        double rawForward = velX * cos - velY * sin;
        double rawStrafe = velX * sin + velY * cos;

        // 5. Giro Independente (Heading PID)
        double errorHeading = MathUtils.angleWrap(targetPoint.heading - robotHeading);
        double rawTurn = headingPID.calculate(-errorHeading, 0);

        // --- FILTRO DE INVERSÃO DE HARDWARE ---
        // Se o robô for para o lado errado no teste da caixa amanhã,
        // mude este valor para -1.0 em vez de mexer na matemática!
        double inversorHardware = 1.0;

        rawForward *= inversorHardware;
        rawStrafe *= inversorHardware;
        rawTurn *= inversorHardware;

        // 6. Atualiza limitadores e Suaviza as potências
        fwdLimiter.setRateLimit(PurePursuitConstants.ACCEL_FWD);
        strLimiter.setRateLimit(PurePursuitConstants.ACCEL_STR);
        turnLimiter.setRateLimit(PurePursuitConstants.ACCEL_TURN);

        return new double[]{
                fwdLimiter.calculate(rawForward),
                strLimiter.calculate(rawStrafe),
                turnLimiter.calculate(rawTurn)
        };
    }

    private Waypoint getLookaheadPoint(double robotX, double robotY) {
        Waypoint bestPoint = null;

        for (int i = lastFoundIndex; i < path.size() - 1; i++) {
            Waypoint start = path.get(i);
            Waypoint end = path.get(i + 1);

            Waypoint intersection = MathUtils.getCircleLineIntersection(
                    robotX, robotY, PurePursuitConstants.LOOKAHEAD_DISTANCE,
                    start.x, start.y, end.x, end.y
            );

            if (intersection != null) {
                double segmentDist = Math.hypot(end.x - start.x, end.y - start.y);
                double interpDist = Math.hypot(intersection.x - start.x, intersection.y - start.y);
                double t = interpDist / segmentDist;

                double headingDiff = MathUtils.angleWrap(end.heading - start.heading);
                intersection.heading = MathUtils.angleWrap(start.heading + headingDiff * t);

                bestPoint = intersection;
                lastFoundIndex = i;
            }
        }
        return bestPoint;
    }

    public boolean isFinished(double robotX, double robotY) {
        if (path.isEmpty()) return true;
        Waypoint last = path.get(path.size() - 1);
        return Math.hypot(robotX - last.x, robotY - last.y) < PurePursuitConstants.END_TOLERANCE;
    }
}