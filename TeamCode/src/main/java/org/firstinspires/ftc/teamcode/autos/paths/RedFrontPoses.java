package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import java.util.List;

public class RedFrontPoses {

    public static final Pose[] POSES = {
            // StartPose
            new Pose(110, 126, Math.toRadians(40)),

            // GoToShoot1
            new Pose(80.134, 90, Math.toRadians(45)),

            // GoToLine1
            new Pose(80.134, 85, Math.toRadians(180)),
            // CatchLine1
            new Pose(114, 85, Math.toRadians(180)),

            // GoToShoot2
            new Pose(80.134, 90, Math.toRadians(48)),

            // GoToLine2
            new Pose(80.134, 61.169, Math.toRadians(180)),
            // CatchLine2
            new Pose(122.776, 61.169, Math.toRadians(180)),

            // EndPose
            new Pose(84, 60, Math.toRadians(90)),

            // GoToLine3
            new Pose(80.134, 38.169, Math.toRadians(180)),
            // CatchLine3
            new Pose(122.776, 38.169, Math.toRadians(180)),

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