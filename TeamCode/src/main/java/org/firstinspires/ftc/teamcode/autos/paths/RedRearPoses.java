package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import org.opencv.core.Mat;

import java.util.List;

public class RedRearPoses {

    public static final Pose[] POSES = {
            new Pose(83.35, 13.399, Math.toRadians(90)),
            new Pose(81.35, 20.547, Math.toRadians(68)),

            new Pose(74.134, 39.169, Math.toRadians(180)),
            new Pose(120.276, 39.169, Math.toRadians(180)),

            new Pose(83.35, 17.547, Math.toRadians(69)),


            new Pose(104.35, 18.547, Math.toRadians(180)),
            new Pose(130.776, 10, Math.toRadians(180)),

            new Pose(85.35, 34, Math.toRadians(90)),

            new Pose(74.134, 63.169, Math.toRadians(180)),
            new Pose(120.276, 63.169, Math.toRadians(180)),

            new Pose(126.276, 67.169, Math.toRadians(90)),

            new Pose(130.276, 28, Math.toRadians(180)),

            new Pose(114.276, 14.5, Math.toRadians(180)),

            new Pose(81.35, 18.547, Math.toRadians(68)),

            new Pose(122.276, 18, Math.toRadians(90)),
    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}
