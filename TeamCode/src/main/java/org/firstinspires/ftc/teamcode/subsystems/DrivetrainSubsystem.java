package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.DataStorage;
import org.firstinspires.ftc.teamcode.utils.Polygon2d;


/**
 * The DrivetrainSubsystem is responsible for the robot's movement and path following.
 * It uses the Pedro Pathing library to control the robot's motion and provides telemetry and visualization.
 */
//@AutoLog
public class DrivetrainSubsystem extends SubsystemBase {
    private final Follower follower;
    private final TelemetryManager telemetry;

    /**
     * Constructs a new DrivetrainSubsystem.
     *
     * @param hardwareMap The hardware map to retrieve hardware devices from.
     * @param telemetry   The telemetry manager for logging.
     */
    public DrivetrainSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        follower = Constants.createFollower(hardwareMap);
        this.telemetry = telemetry;
        Drawing.init();
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();

    }


    /**
     * Gets the Follower instance used for path following.
     * @return The Follower instance.
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Drives the robot using Pedro Pathing's internal vector system.
     * This is the cleanest way to do manual control without declaring motors.
     *
     * @param strafe Speed in x direction (sideways)
     * @param forward Speed in y direction (forward)
     * @param turn Speed of rotation
     */
    public void driveRobotCentric(double strafe, double forward, double turn) {

        follower.setTeleOpDrive(forward, strafe, turn, true);

        follower.update();
    }

    public boolean isRobotStopped() {
        double linearVelocity = follower.getVelocity().getMagnitude();
        double angularVelocity = Math.toDegrees(Math.abs(follower.getAngularVelocity()));

        return linearVelocity < 2.0 && angularVelocity < 3.0;
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        follower.breakFollowing();
        follower.setTeleOpDrive(0, 0, 0, true);
    }

    // 1. Cria as zonas UMA VEZ na memória para otimização máxima
    private final Polygon2d triangleBig = new Polygon2d(new Translation2d(72, 72), new Translation2d(144, 144), new Translation2d(0, 144));
    private final Polygon2d triangleSmall = new Polygon2d(new Translation2d(72, 30), new Translation2d(44, 0), new Translation2d(100, 0));

    // 2. A trava de segurança do piloto (Começa desligada)
    private boolean autoShootEnabled = false;

    // Método para o piloto apertar um botão e ligar/desligar o modo
    public void toggleAutoShoot() {
        autoShootEnabled = !autoShootEnabled;
    }

    public boolean isAutoShootEnabled() {
        return autoShootEnabled;
    }

    // 3. O "Radar" que diz se estamos na área de tiro
    public boolean isInShootingZone() {
        Translation2d currentPos = new Translation2d(follower.getPose().getX(), follower.getPose().getY());
        return triangleBig.containsPoint(currentPos) || triangleSmall.containsPoint(currentPos);
    }

    // Variável para guardar se o chassi está travado no alvo
    private boolean isAimLocked = false;

    // O Comando vai usar isso para avisar o chassi
    public void setAimLocked(boolean locked) {
        this.isAimLocked = locked;
    }

    // O Trigger vai usar isso para ler o status
    public boolean isAimLocked() {
        return this.isAimLocked;
    }

    /**
     * This method is called periodically to update the subsystem's state, including the follower,
     * telemetry, and dashboard visualizations.
     */
    @Override
    public void periodic() {

        follower.update();
        telemetry.addData("Robot pose",follower.getPose());
        Drawing.drawRobot(follower.getPose());
        Drawing.sendPacket();
        Drawing.drawDebug(follower);
        DataStorage.actualPose = follower.getPose();

        //Polygon2d triangleBig = new Polygon2d(new Translation2d(72, 72), new Translation2d(144, 144), new Translation2d(0, 144));
        //Polygon2d triangleSmall = new Polygon2d(new Translation2d(72, 30), new Translation2d(44, 0), new Translation2d(100, 0));

        //telemetry.addData("Inside big triangle", triangleBig.containsPoint(new Translation2d(follower.getPose().getX(), follower.getPose().getY())));
        //telemetry.addData("Inside Small triangle", triangleSmall.containsPoint(new Translation2d(follower.getPose().getX(), follower.getPose().getY())));

    }
}

/**
 * The Drawing class handles the visualization of the robot and its path on the Panels Dashboard.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
class Drawing {
    public static final double ROBOT_RADIUS = 8; // Robot radius in inches
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "#008000", "#3F51B5", 0.0
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.0
    );

    /**
     * Initializes the Panels Field with default FTC offsets.
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Draws debug information from the Follower, including the current path and pose history.
     *
     * @param follower The Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * Draws a representation of the robot at a specified Pose with a given style.
     * The robot's heading is indicated by a line.
     *
     * @param pose  The Pose to draw the robot at.
     * @param style The style parameters for drawing.
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Draws a representation of the robot at a specified Pose using the default style.
     *
     * @param pose The Pose to draw the robot at.
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * Draws a Path with a specified style.
     *
     * @param path  The Path to draw.
     * @param style The style parameters for drawing.
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * Draws all Paths in a PathChain with a specified style.
     *
     * @param pathChain The PathChain to draw.
     * @param style     The style parameters for drawing.
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * Draws the robot's pose history with a specified style.
     *
     * @param poseTracker The PoseHistory containing the robot's path.
     * @param style       The style parameters for drawing.
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * Draws the robot's pose history using the default style.
     *
     * @param poseTracker The PoseHistory containing the robot's path.
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * Sends the current drawing packet to the FTControl Panels dashboard.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}