package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {

            new Pose(63.411, 13.399, Math.toRadians(90)),
            new Pose(60.335, 18.547, Math.toRadians(111)),

            new Pose(64.134, 38.169, Math.toRadians(0)),
            new Pose(24, 38.169, Math.toRadians(0)),

            new Pose(60.335, 17.547, Math.toRadians(109)),


            new Pose(41, 19.547, Math.toRadians(0)),
            new Pose(24, 13, Math.toRadians(0)),

            new Pose(60.335, 34, Math.toRadians(90)),

            new Pose(64.134, 63.169, Math.toRadians(0)),
            new Pose(26, 63.169, Math.toRadians(0)),

            new Pose(18, 64.169, Math.toRadians(90)),

            new Pose(20, 28, Math.toRadians(0)),

            new Pose(20, 14.5, Math.toRadians(0)),

            new Pose(60.335, 18.547, Math.toRadians(113)),

            new Pose(120.276, 18, Math.toRadians(90)),


    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
