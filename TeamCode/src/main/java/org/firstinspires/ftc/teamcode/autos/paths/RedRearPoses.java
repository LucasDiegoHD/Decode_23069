package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

import java.util.List;

public class RedRearPoses {

    public static final Pose[] POSES = {
            new Pose(82.411, 8.399, Math.toRadians(90)),
            new Pose(83.35, 14.547, Math.toRadians(68)),

            new Pose(94.134, 34.169, Math.toRadians(180)),
            new Pose(126.276, 34.169, Math.toRadians(180)),
            new Pose(86.335, 12.547, Math.toRadians(67.5)),


            new Pose(83.35, 14.547, Math.toRadians(180)),
            new Pose(128.276, 12, Math.toRadians(180)),

            new Pose(106, 34, Math.toRadians(90)),

            new Pose(94.134, 58.169, Math.toRadians(180)),
            new Pose(132.276, 58.169, Math.toRadians(180)),

            new Pose(128.276, 64.169, Math.toRadians(90)),

    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}
