package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import java.util.List;

public class BlueFrontPoses {

    // AVISO: Ajuste estas coordenadas para o lado do Triângulo Grande!
    public static final Pose[] POSES = {

            new Pose(15, 8.5, Math.toRadians(90)),

            new Pose(30, 12, Math.toRadians(112)),

            new Pose(45, 48, Math.toRadians(0)),

            new Pose(11.776, 48, Math.toRadians(0)),

            new Pose(30, 12, Math.toRadians(112)),

            new Pose(45, 55, Math.toRadians(0)),

            new Pose(11.776, 55, Math.toRadians(0)),

            new Pose(50, 12, Math.toRadians(90)),

            new Pose(45, 60, Math.toRadians(0)),

            new Pose(11.776, 60, Math.toRadians(0)),

            new Pose(30, 30, Math.toRadians(90)),

            new Pose(15, 15, Math.toRadians(0))
    };

    public static Pose getPose(PosesNames name) {
        // Garante que não estoure o array se o Enum for maior
        if (name.ordinal() >= POSES.length) return POSES[0];
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}