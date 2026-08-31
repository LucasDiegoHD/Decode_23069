package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for geometric and mathematical calculations.
 */
public class MathUtils {

    /**
     * Normalizes an angle to be within the range (-PI, PI].
     * @param angle The angle in radians.
     * @return The normalized angle.
     */
    public static double angleWrap(double angle) {
        while (angle < -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    /**
     * Calculates the intersection points between a circle (robot lookahead) and a line segment.
     * @param robotX Robot X position.
     * @param robotY Robot Y position.
     * @param radius Lookahead radius.
     * @param startX Line segment start X.
     * @param startY Line segment start Y.
     * @param endX Line segment end X.
     * @param endY Line segment end Y.
     * @return The intersection Waypoint (without heading) closest to the end point, or null if no intersection.
     */
    public static Waypoint getCircleLineIntersection(double robotX, double robotY, double radius, double startX, double startY, double endX, double endY) {
        double baX = endX - startX;
        double baY = endY - startY;
        double caX = robotX - startX;
        double caY = robotY - startY;

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;

        if (disc < 0) {
            return null; // No intersection
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        List<Waypoint> validIntersections = new ArrayList<>();

        if (abScalingFactor1 >= 0 && abScalingFactor1 <= 1) {
            validIntersections.add(new Waypoint(startX + baX * abScalingFactor1, startY + baY * abScalingFactor1, 0));
        }
        if (abScalingFactor2 >= 0 && abScalingFactor2 <= 1) {
            validIntersections.add(new Waypoint(startX + baX * abScalingFactor2, startY + baY * abScalingFactor2, 0));
        }

        if (validIntersections.isEmpty()) return null;

        // Find the intersection closest to the end point (to ensure forward progress)
        Waypoint bestPoint = null;
        double minDist = Double.MAX_VALUE;

        for (Waypoint p : validIntersections) {
            double dist = Math.hypot(p.x - endX, p.y - endY);
            if (dist < minDist) {
                minDist = dist;
                bestPoint = p;
            }
        }

        return bestPoint;
    }
}