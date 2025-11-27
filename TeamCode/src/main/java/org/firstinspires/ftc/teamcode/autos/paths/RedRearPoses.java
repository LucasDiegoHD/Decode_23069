package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

import java.util.List;

public class RedRearPoses {

    public static final Pose[] POSES = {
            new Pose(82.411, 8.399, Math.toRadians(90)),
            new Pose(84.335, 10.147, Math.toRadians(68)),

            new Pose(94.134, 34.169, Math.toRadians(180)),
            new Pose(134.776, 34.169, Math.toRadians(180)),
            new Pose(86.335, 10.147, Math.toRadians(68)),


            new Pose(136, 23, Math.toRadians(110)),
            new Pose(136, 6, Math.toRadians(90)),

            new Pose(106, 34, Math.toRadians(90)),

    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}
