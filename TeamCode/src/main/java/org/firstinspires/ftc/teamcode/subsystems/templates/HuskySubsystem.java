package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Optional;

/**
 * HuskyLens Smart Vision Subsystem Template
 *
 * Overview:
 * Interfaces with a DFRobot HuskyLens AI Camera over the I2C bus to provide real-time
 * object recognition, tracking, and optical distance estimation for game artifacts
 * (e.g., Purple and Green sample/scoring elements).
 *
 * Hardware Configuration:
 * - HuskyLens Sensor: "husky" (I2C Bus)
 *
 * Performance and Anti-Lag Strategy:
 * - I2C Rate Limiting (Time-Slicing): Physical I2C block queries are throttled to
 *   a fixed interval (20 FPS / 50ms) in periodic(). Calls to query methods read
 *   from an in-memory cache at near-zero computational overhead (~0.001 ms).
 * - Color Filtering: Rejects foreign color IDs, accepting only trained artifact IDs.
 * - Geometric Aspect Ratio Filter: Rejects thin lines and reflections by enforcing
 *   bounding box aspect ratios between 0.6 and 1.6.
 * - Area Priority: Selects the largest candidate block (closest physical target).
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
public class HuskySubsystem extends SubsystemBase {

    private final HuskyLens huskyLens;
    private final TelemetryManager telemetry;
    private HuskyLens.Block lastValidBlock = null;

    // --- Anti-Lag Cache and Timing ---
    private HuskyLens.Block[] latestBlocks = new HuskyLens.Block[0];
    private long lastHuskyReadTime = 0;
    private static final long HUSKY_READ_INTERVAL_MS = 50; // Reads sensor every 50ms (20 FPS)

    /**
     * Constructs a new HuskySubsystem, verifies sensor communication, and selects
     * the color recognition algorithm.
     *
     * @param hardwareMap Robot hardware map for device retrieval.
     * @param telemetry   Telemetry manager for diagnostic logging.
     */
    public HuskySubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;
        this.huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "ERROR: Device not connected / unresponsive!");
        } else {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }
    }

    /**
     * Returns the most relevant target artifact from cached sensor detections.
     * Applies color ID filtering, minimum pixel size thresholds, and aspect ratio checks,
     * returning the block with the largest bounding area.
     *
     * @return Optional containing the best matching HuskyLens.Block, or empty if none found.
     */
    public Optional<HuskyLens.Block> getClosestAnyArtifact() {
        // Read from in-memory cache to prevent blocking the main control loop
        HuskyLens.Block[] blocks = latestBlocks;

        if (blocks == null || blocks.length == 0) {
            return Optional.empty();
        }

        HuskyLens.Block bestBlock = null;
        double maxArea = -1;

        for (HuskyLens.Block block : blocks) {
            // 1. Color ID Filter
            boolean isValidId = (block.id == HuskyConstants.COLOR_ID_PURPLE ||
                    block.id == HuskyConstants.COLOR_ID_GREEN ||
                    block.id == HuskyConstants.COLOR_ID_GREEN2 ||
                    block.id == HuskyConstants.COLOR_ID_PURPLE2);

            if (!isValidId) continue;

            // 2. Minimum Size Noise Filter
            if (block.width < 15 || block.height < 15) continue;

            // 3. Aspect Ratio Geometry Filter (Rejects thin lines/reflections)
            double ratio = (double) block.width / block.height;
            if (ratio < 0.6 || ratio > 1.6) continue;

            // 4. Area Criterion: Select object with largest bounding box (closest to camera)
            double area = block.width * block.height;
            if (area > maxArea) {
                maxArea = area;
                bestBlock = block;
            }
        }

        if (bestBlock != null) {
            lastValidBlock = bestBlock;
            return Optional.of(bestBlock);
        }

        return Optional.empty();
    }

    /**
     * Estimates the optical distance in inches from the camera to the specified block.
     *
     * @param block The detected HuskyLens.Block.
     * @return Estimated distance in inches, or 0.0 if block is null.
     */
    public double getDistanceToBlock(HuskyLens.Block block) {
        if (block == null) return 0;
        return (HuskyConstants.ARTIFACT_REAL_WIDTH_INCHES * HuskyConstants.FOCAL_LENGTH_PIXELS) / block.width;
    }

    /**
     * Periodic background execution. Polls physical sensor data at rate-limited intervals (20 FPS)
     * and publishes diagnostic telemetry.
     */
    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastHuskyReadTime >= HUSKY_READ_INTERVAL_MS) {
            latestBlocks = huskyLens.blocks(); // Physical I2C transaction
            lastHuskyReadTime = currentTime;
        }

        if (lastValidBlock != null) {
            telemetry.addData("Husky ID", lastValidBlock.id);
            telemetry.addData("Husky X", lastValidBlock.x);
            telemetry.addData("Husky Dist (in)", getDistanceToBlock(lastValidBlock));
        } else {
            telemetry.addData("Husky", "Searching...");
        }
    }
}