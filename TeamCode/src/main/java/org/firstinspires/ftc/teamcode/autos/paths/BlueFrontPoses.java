package org.firstinspires.ftc.teamcode.autos.paths;

import com.pedropathing.geometry.Pose;
import java.util.List;

public class BlueFrontPoses {
    public static final Pose[] POSES = {
            // StartPose
            new Pose(34, 126, Math.toRadians(130)),

            // GoToShoot1
            new Pose(59.134, 79, Math.toRadians(122)),

            // GoToLine1
            new Pose(58.134, 88, Math.toRadians(0)),
            // CatchLine1
            new Pose(26, 88, Math.toRadians(0)),

            // GoToShoot2
            new Pose(59.134, 79, Math.toRadians(120)),

            // GoToLine2
            new Pose(58.134, 65.169, Math.toRadians(0)),
            // CatchLine2
            new Pose(20.776, 65.169, Math.toRadians(0)),

            // EndPose
            new Pose(50, 60, Math.toRadians(90)),

            // GoToLine3
            new Pose(58.134, 68.669, Math.toRadians(0)),
            // CatchLine3
            new Pose(14.774, 68.669, Math.toRadians(285)),

            // GatePose
            new Pose(14.774, 69.169, Math.toRadians(0)),

    };

    public static Pose getPose(PosesNames name) {
        if (name.ordinal() >= POSES.length) return POSES[0];
        return POSES[name.ordinal()];
    }

    public static List<Pose> asList() {
        return List.of(POSES);
    }
}