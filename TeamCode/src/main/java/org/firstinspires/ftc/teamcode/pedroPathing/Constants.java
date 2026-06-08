package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static class Drivetrain {
        public static String LEFT_FRONT_MOTOR = "leftFront";
        public static String RIGHT_FRONT_MOTOR = "rightFront";
        public static String LEFT_REAR_MOTOR = "leftRear";
        public static String RIGHT_REAR_MOTOR = "rightRear";
        public static String PINPOINT_LOCALIZER = "pinpoint";
    }

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8)
            .forwardZeroPowerAcceleration(-73.315092662612045)
            .lateralZeroPowerAcceleration(-97.42340280933647)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0000)
            .translationalPIDFCoefficients(
                    new PIDFCoefficients(5, 0.01, 0.3, 0.3)
            )
            .headingPIDFCoefficients(
                    new PIDFCoefficients(1.4, 0.0, 0.3, 0.02)
            )
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.65, 0.001, 0.09, 0.5, 0.02)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName(Drivetrain.LEFT_FRONT_MOTOR)
            .leftRearMotorName(Drivetrain.LEFT_REAR_MOTOR)
            .rightFrontMotorName(Drivetrain.RIGHT_FRONT_MOTOR)
            .rightRearMotorName(Drivetrain.RIGHT_REAR_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.11611423342248)
            .yVelocity(70.88718660609929)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.5)
            .strafePodX(1.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(Drivetrain.PINPOINT_LOCALIZER)
            .yawScalar(1.00474)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Constraints padrão — usados no TeleOp e como base
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,  // tValue
            2.0,   // velocity (in/s)
            1.5,   // translational (inches)
            0.04,  // heading (rad)
            250,   // timeout (ms)
            1.5,   // brakingStrength
            10,    // bezier limit
            0.4    // brakingStart
    );

    // Constraints para movimentos de TRAVESSIA no autônomo.
    // O robô não precisa parar com precisão — só precisa chegar perto
    // e continuar. Muito mais rápido pois não espera desacelerar.
    public static PathConstraints autoTransitConstraints = new PathConstraints(
            0.95,  // tValue — termina aos 95% do caminho, não espera o fim
            10.0,   // velocity — pode ainda estar rápido ao "terminar"
            6.0,   // translational — aceita até 3 inches de erro
            0.1,   // heading — aceita até ~5.7° de erro
            100,   // timeout — só 100ms de correção, não 250
            2.0,
            5,
            0.1
    );

    // Constraints para poses de TIRO — precisa parar com precisão
    public static PathConstraints autoShootConstraints = new PathConstraints(
            0.97,  // tValue — percorre quase tudo
            8.0,   // velocity — pode estar um pouco mais rápido que antes
            1.5,   // translational — 1.5 inches de tolerância
            0.05,  // heading — ~2.9°
            300,   // timeout — 150ms é suficiente
            2.5,
            10,
            0.4
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

    public static double TIME_BETWEEN_LINES = 2000;
}