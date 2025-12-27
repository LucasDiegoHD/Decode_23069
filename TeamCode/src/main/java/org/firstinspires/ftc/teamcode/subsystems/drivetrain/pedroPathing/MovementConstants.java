package org.firstinspires.ftc.teamcode.subsystems.drivetrain.pedroPathing;

import com.pedropathing.paths.PathConstraints;

/**
 * Perfis de movimento para o Pedro Pathing.
 * Precisamos ajustar o T_Scaling para força e o Timeout para segurança.
 */

public class MovementConstants {

    public static PathConstraints FAST_CONSTRAINTS = new PathConstraints(0.8, 4.0);
    public static PathConstraints MEDIUM_CONSTRAINTS = new PathConstraints(0.7, 3.0);
    public static PathConstraints PRECISION_CONSTRAINTS = new PathConstraints(0.4, 2.0);
    public static PathConstraints PICKUP_CONSTRAINTS = new PathConstraints(0.5, 2.5);
}