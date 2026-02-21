package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import java.util.List;

public class BlueFrontPoses {
    public static final Pose[] POSES = {
            // StartPose
            new Pose(34, 126, Math.toRadians(130)),

            // GoToShoot1
            new Pose(56.134, 90, Math.toRadians(135)),

            // GoToLine1
            new Pose(56.134, 85, Math.toRadians(0)),
            // CatchLine1
            new Pose(30, 85, Math.toRadians(0)),

            // GoToShoot2
            new Pose(56.134, 90, Math.toRadians(138)),

            // GoToLine2
            new Pose(56.134, 61.169, Math.toRadians(0)),
            // CatchLine2
            new Pose(22.776, 61.169, Math.toRadians(0)),

            // EndPose
            new Pose(50, 60, Math.toRadians(90)),

            // GoToLine3
            new Pose(56.134, 38.169, Math.toRadians(0)),
            // CatchLine3
            new Pose(22.776, 38.169, Math.toRadians(0)),

            // GatePose
            new Pose(12.774, 64.169, Math.toRadians(90)),
    };

    public static Pose getPose(PosesNames name) {
        if (name.ordinal() >= POSES.length) return POSES[0];
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}