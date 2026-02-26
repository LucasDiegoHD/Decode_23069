package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {

            new Pose(62.411, 8.399, Math.toRadians(90)),
            new Pose(60.335, 14.547, Math.toRadians(113.5)),

            new Pose(52.134, 35.169, Math.toRadians(0)),
            new Pose(21, 35.169, Math.toRadians(0)),

            new Pose(60.335, 14.547, Math.toRadians(111.5)),


            new Pose(41, 18.547, Math.toRadians(0)),
            new Pose(12, 7, Math.toRadians(0)),

            new Pose(60.335, 34, Math.toRadians(90)),

            new Pose(52.134, 59.569, Math.toRadians(0)),
            new Pose(21, 59.169, Math.toRadians(0)),

            new Pose(13.276, 64.169, Math.toRadians(90)),

            new Pose(12, 28, Math.toRadians(0)),

            new Pose(12, 13, Math.toRadians(0)),


    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
