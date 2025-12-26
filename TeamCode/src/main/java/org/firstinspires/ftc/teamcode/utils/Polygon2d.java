package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Translation2d;

public class Polygon2d {
    private final Translation2d[] vertices;

    public Polygon2d(Translation2d... vertices) {
        this.vertices = vertices;
    }

    public Translation2d[] getVertices() {
        return vertices;
    }

    // Algoritmo do ray-casting para saber se o ponto está dentro
    public boolean containsPoint(Translation2d point) {
        boolean inside = false;
        int j = vertices.length - 1;

        for (int i = 0; i < vertices.length; i++) {
            double xi = vertices[i].getX();
            double yi = vertices[i].getY();
            double xj = vertices[j].getX();
            double yj = vertices[j].getY();

            boolean intersect =
                    ((yi > point.getY()) != (yj > point.getY()))
                            && (point.getX() < (xj - xi) * (point.getY() - yi) / (yj - yi) + xi);

            if (intersect) inside = !inside;
            j = i;
        }

        return inside;
    }
}
