package org.firstinspires.ftc.teamcode.utils;

/**
 * Represents a point on the field with a target heading.
 * Used for the custom Pure Pursuit algorithm.
 */
public class Waypoint {
    public double x;
    public double y;
    public double heading; // in radians

    /**
     * Constructor for Waypoint.
     * @param x X coordinate in inches.
     * @param y Y coordinate in inches.
     * @param heading Heading in radians.
     */
    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}