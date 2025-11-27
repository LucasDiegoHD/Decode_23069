package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {

            new Pose(62.411, 8.399, Math.toRadians(90)),
            new Pose(68.335, 10.147, Math.toRadians(113)),

            new Pose(52.134, 34.169, Math.toRadians(0)),
            new Pose(11.776, 33.669, Math.toRadians(0)),
            new Pose(64.335, 10.147, Math.toRadians(113)),


            new Pose(8, 23, Math.toRadians(70)),
            new Pose(8, 5, Math.toRadians(90)),

            new Pose(38, 34, Math.toRadians(90)),

    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
