package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.controller.PIDController;
import java.util.ArrayList;

/**
 * Custom Pure Pursuit Controller implementation using 3 PIDs.
 * Controls Forward, Strafe, and Heading independently.
 */
public class PurePursuitController {

    private PIDController forwardPID;
    private PIDController strafePID;
    private PIDController headingPID;

    private ArrayList<Waypoint> path = new ArrayList<>();
    private int lastFoundIndex = 0;

    // Tuning Parameters
    private double lookaheadDistance = 12.0; // inches
    private double endTolerance = 1.0; // inches

    public PurePursuitController(PIDController fwd, PIDController str, PIDController head) {
        this.forwardPID = fwd;
        this.strafePID = str;
        this.headingPID = head;
    }

    public void setPath(ArrayList<Waypoint> newPath) {
        this.path = newPath;
        this.lastFoundIndex = 0;
    }

    /**
     * Updates the controller and calculates motor powers.
     * @param robotX Current X position.
     * @param robotY Current Y position.
     * @param robotHeading Current Heading (radians).
     * @return double[] {forwardPower, strafePower, turnPower} (Robot Centric)
     */
    public double[] update(double robotX, double robotY, double robotHeading) {
        Waypoint targetPoint = getLookaheadPoint(robotX, robotY);

        // If no lookahead point is found (end of path), aim for the very last waypoint
        if (targetPoint == null) {
            targetPoint = path.get(path.size() - 1);
        }

        // Calculate Global Error
        double errorX = targetPoint.x - robotX;
        double errorY = targetPoint.y - robotY;

        // Check if we are close to the end to stop
        double distToTarget = Math.hypot(errorX, errorY);
        if (lastFoundIndex >= path.size() - 2 && distToTarget < endTolerance) {
            return new double[]{0, 0, 0};
        }

        // Rotate error to Robot Centric (Field Centric -> Robot Centric transformation)
        double sin = Math.sin(-robotHeading);
        double cos = Math.cos(-robotHeading);

        double errorForward = errorX * cos - errorY * sin; // x*cos - y*sin (Standard rotation matrix)
        double errorStrafe = errorX * sin + errorY * cos;  // x*sin + y*cos

        // Calculate Heading Error
        double errorHeading = MathUtils.angleWrap(targetPoint.heading - robotHeading);

        // Calculate PID Outputs
        // We want error to be 0, so setpoint is 0 and measurement is -error
        double powerForward = forwardPID.calculate(-errorForward, 0);
        double powerStrafe = strafePID.calculate(-errorStrafe, 0);
        double powerTurn = headingPID.calculate(-errorHeading, 0);

        return new double[]{powerForward, powerStrafe, powerTurn};
    }

    private Waypoint getLookaheadPoint(double robotX, double robotY) {
        Waypoint bestPoint = null;

        for (int i = lastFoundIndex; i < path.size() - 1; i++) {
            Waypoint start = path.get(i);
            Waypoint end = path.get(i + 1);

            Waypoint intersection = MathUtils.getCircleLineIntersection(robotX, robotY, lookaheadDistance, start.x, start.y, end.x, end.y);

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
        return Math.hypot(robotX - last.x, robotY - last.y) < endTolerance;
    }
}