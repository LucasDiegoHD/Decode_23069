package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

import java.util.List;

public class RedRearPoses {

    public static final Pose[] POSES = {
            new Pose(83.35, 12.399, Math.toRadians(90)),
            new Pose(81.35, 16.547, Math.toRadians(66)),

            new Pose(96.134, 36.169, Math.toRadians(180)),
            new Pose(126.276, 36.169, Math.toRadians(180)),

            new Pose(83.35, 16.547, Math.toRadians(69)),


            new Pose(103.35, 20.547, Math.toRadians(180)),
            new Pose(133.776, 10, Math.toRadians(180)),

            new Pose(85.35, 34, Math.toRadians(90)),

            new Pose(96.134, 58.169, Math.toRadians(180)),
            new Pose(126.276, 58.169, Math.toRadians(180)),

            new Pose(130.276, 64.169, Math.toRadians(90)),

            new Pose(132.276, 28, Math.toRadians(180)),

            new Pose(126.276, 14, Math.toRadians(180)),
    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}
