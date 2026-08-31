package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import java.util.List;

public class RedFrontPoses {

    public static final Pose[] POSES = {
            // StartPose
            new Pose(110, 126, Math.toRadians(40)),

            // GoToShoot1
            new Pose(78.634, 83, Math.toRadians(48)),

            // GoToLine1
            new Pose(90.134, 88, Math.toRadians(180)),
            // CatchLine1
            new Pose(116, 88, Math.toRadians(180)),

            // GoToShoot2'
            new Pose(78.634, 83, Math.toRadians(50)),

            // GoToLine2
            new Pose(90.134, 65.169, Math.toRadians(180)),
            // CatchLine2
            new Pose(118.776, 65.169, Math.toRadians(180)),

            // EndPose
            new Pose(84, 60, Math.toRadians(90)),

            // GoToLine3
            new Pose(120.134, 63.169, Math.toRadians(180)),
            // CatchLine3
            new Pose(130, 63.169, Math.toRadians(255)),

            // GatePose
            new Pose(130, 73.169, Math.toRadians(180)),

            new Pose(130, 63.169, Math.toRadians(255)),

    };

    public static Pose getPose(PosesNames name) {
        if (name.ordinal() >= POSES.length) return POSES[0];
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}