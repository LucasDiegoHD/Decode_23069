package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class BlueRearPoses {


    public static final Pose[] POSES = {

            new Pose(62.411, 8.399, Math.toRadians(90)),
            new Pose(60.335, 14.547, Math.toRadians(112)),

            new Pose(52.134, 34.169, Math.toRadians(0)),
            new Pose(11.776, 33.669, Math.toRadians(0)),
            new Pose(64.335, 8.547, Math.toRadians(112)),


            new Pose(16, 14.547, Math.toRadians(0)),
            new Pose(16, 12, Math.toRadians(0)),

            new Pose(38, 34, Math.toRadians(90)),

            new Pose(52.134, 58.169, Math.toRadians(0)),
            new Pose(11.776, 57.669, Math.toRadians(0)),

            new Pose(16.276, 64.169, Math.toRadians(90)),


    };

    public static Pose getPose(PosesNames name) {
        return POSES[name.ordinal()];
    }
    public static List<Pose> asList() {
        return List.of(POSES);
    }


}
