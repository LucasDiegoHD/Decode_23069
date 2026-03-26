package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import java.util.ArrayList;

/**
 * Custom Pure Pursuit Controller implementation using 3 PIDs.
 * Controls Forward, Strafe, and Heading independently.
 * Integrado em tempo real com PurePursuitConstants para tuning no Dashboard.
 */
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
        // Zera os limitadores sempre que iniciar um caminho novo para não dar tranco
        fwdLimiter.reset(0);
        strLimiter.reset(0);
        turnLimiter.reset(0);
    }

    /**
     * Updates the controller and calculates motor powers.
     * @param robotX Current X position.
     * @param robotY Current Y position.
     * @param robotHeading Current Heading (radians).
     * @return double[] {forwardPower, strafePower, turnPower} (Robot Centric)
     */
    public double[] update(double robotX, double robotY, double robotHeading) {

        if (path == null || path.isEmpty()) {
            return new double[]{0, 0, 0};
        }

        Waypoint targetPoint = getLookaheadPoint(robotX, robotY);

        if (targetPoint == null) {
            targetPoint = path.get(path.size() - 1);
        }

        double errorX = targetPoint.x - robotX;
        double errorY = targetPoint.y - robotY;

        double distToTarget = Math.hypot(errorX, errorY);
        if (lastFoundIndex >= path.size() - 2 && distToTarget < PurePursuitConstants.END_TOLERANCE) {
            return new double[]{0, 0, 0};
        }

        double sin = Math.sin(-robotHeading);
        double cos = Math.cos(-robotHeading);

        double errorForward = errorX * cos - errorY * sin;
        double errorStrafe = errorX * sin + errorY * cos;
        double errorHeading = MathUtils.angleWrap(targetPoint.heading - robotHeading);

        // Atualiza os limites de aceleração em tempo real (caso mude no Dashboard)
        fwdLimiter.setRateLimit(PurePursuitConstants.ACCEL_FWD);
        strLimiter.setRateLimit(PurePursuitConstants.ACCEL_STR);
        turnLimiter.setRateLimit(PurePursuitConstants.ACCEL_TURN);

        // 1. Calcula o PID (Potência Bruta)
        double rawForward = forwardPID.calculate(-errorForward, 0);
        double rawStrafe = strafePID.calculate(-errorStrafe, 0);
        double rawTurn = headingPID.calculate(-errorHeading, 0);

        // 2. Passa a Potência Bruta pelo Limitador (Potência Suavizada)
        double smoothForward = fwdLimiter.calculate(rawForward);
        double smoothStrafe = strLimiter.calculate(rawStrafe);
        double smoothTurn = turnLimiter.calculate(rawTurn);

        // Retorna a potência que não vai fazer o robô derrapar
        return new double[]{smoothForward, smoothStrafe, smoothTurn};
    }

    private Waypoint getLookaheadPoint(double robotX, double robotY) {
        Waypoint bestPoint = null;

        for (int i = lastFoundIndex; i < path.size() - 1; i++) {
            Waypoint start = path.get(i);
            Waypoint end = path.get(i + 1);

            // Usa a constante do Dashboard em tempo real
            Waypoint intersection = MathUtils.getCircleLineIntersection(
                    robotX, robotY, PurePursuitConstants.LOOKAHEAD_DISTANCE,
                    start.x, start.y, end.x, end.y
            );

            if (intersection != null) {
                // Linearly interpolate the heading based on distance
                double segmentDist = Math.hypot(end.x - start.x, end.y - start.y);
                double interpDist = Math.hypot(intersection.x - start.x, intersection.y - start.y);
                double t = interpDist / segmentDist;

                double headingDiff = MathUtils.angleWrap(end.heading - start.heading);
                intersection.heading = MathUtils.angleWrap(start.heading + headingDiff * t);

                bestPoint = intersection;
                lastFoundIndex = i; // Advance the index so we don't go backwards
            }
        }
        return bestPoint;
    }

    public boolean isFinished(double robotX, double robotY) {
        if (path.isEmpty()) return true;
        Waypoint last = path.get(path.size() - 1);

        // Usa a constante do Dashboard para verificar se já chegou
        return Math.hypot(robotX - last.x, robotY - last.y) < PurePursuitConstants.END_TOLERANCE;
    }
}