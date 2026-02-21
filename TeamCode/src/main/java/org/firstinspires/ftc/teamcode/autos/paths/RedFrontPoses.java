package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import java.util.List;

public class RedFrontPoses {

    public static final Pose[] POSES = {
            // StartPose
            new Pose(110, 126, Math.toRadians(40)),

            // GoToShoot1
            new Pose(84.134, 94, Math.toRadians(46)),

            // GoToLine1
            new Pose(80.134, 86, Math.toRadians(180)),
            // CatchLine1
            new Pose(110, 86, Math.toRadians(180)),

            // GoToShoot2
            new Pose(80.134, 90, Math.toRadians(48)),

            // GoToLine2
            new Pose(80.134, 62.169, Math.toRadians(180)),
            // CatchLine2
            new Pose(115.776, 62.169, Math.toRadians(180)),

            // EndPose
            new Pose(84, 60, Math.toRadians(90)),

            // GoToLine3
            new Pose(80.134, 40.169, Math.toRadians(180)),
            // CatchLine3
            new Pose(115.776, 40.169, Math.toRadians(180)),

            // GatePose
            new Pose(123.776, 64.169, Math.toRadians(90)),
    };

    public static Pose getPose(PosesNames name) {
        if (name.ordinal() >= POSES.length) return POSES[0];
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}