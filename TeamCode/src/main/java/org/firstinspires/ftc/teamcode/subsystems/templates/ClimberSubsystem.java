package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Climber Subsystem Template
 *
 * Overview:
 * Controls the robot's hanging and climbing winch mechanism using four Continuous
 * Rotation Servos (CRServos). The mechanism extends lift arms and retracts under
 * load to elevate the robot during endgame scoring.
 *
 * Hardware Configuration:
 * - Left Outer Servo:  "leftLClimber"
 * - Left Inner Servo:  "leftRClimber"
 * - Right Inner Servo: "rightLClimber"
 * - Right Outer Servo: "rightRClimber"
 *
 * Control Strategy and Safety:
 * - Direct normalized power control within the [-1.0, 1.0] range.
 * - Automatic zero-power initialization upon instantiation.
 * - Hardware power bounds clamping on manual joystick input.
 *
 * @author LucasDiegoHD - Team #23069
 * @version 1.0
 */
public class ClimberSubsystem extends SubsystemBase {

    private final CRServo leftServol;
    private final CRServo leftServor;
    private final CRServo rightServor;
    private final CRServo rightServol;

    /**
     * Constructs a new ClimberSubsystem and initializes all 4 CRServos.
     *
     * @param hardwareMap Robot hardware map for device retrieval.
     */
    public ClimberSubsystem(HardwareMap hardwareMap) {
        leftServol = hardwareMap.get(CRServo.class, ClimberConstants.LEFT_SERVOL_NAME);
        leftServor = hardwareMap.get(CRServo.class, ClimberConstants.LEFT_SERVOR_NAME);
        rightServor = hardwareMap.get(CRServo.class, ClimberConstants.RIGHT_SERVOR_NAME);
        rightServol = hardwareMap.get(CRServo.class, ClimberConstants.RIGHT_SERVOL_NAME);

        leftServol.setDirection(DcMotorSimple.Direction.FORWARD);
        rightServol.setDirection(DcMotorSimple.Direction.FORWARD);
        leftServor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightServor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Safety measure: Ensure all servos start at 0 power
        stop();
    }

    /**
     * Commands all climber servos to rotate upwards, extending the climbing arm/hook.
     */
    public void esticar() {
        leftServol.setPower(-ClimberConstants.POWER_UP);
        rightServol.setPower(-ClimberConstants.POWER_UP);
        leftServor.setPower(-ClimberConstants.POWER_UP);
        rightServor.setPower(-ClimberConstants.POWER_UP);
    }

    /**
     * Alias for esticar() - Extends the climber arms upwards.
     */
    public void extend() {
        esticar();
    }

    /**
     * Commands all climber servos to rotate downwards, retracting/pulling the robot up.
     */
    public void recolher() {
        leftServol.setPower(-ClimberConstants.POWER_DOWN);
        rightServol.setPower(-ClimberConstants.POWER_DOWN);
        leftServor.setPower(-ClimberConstants.POWER_DOWN);
        rightServor.setPower(-ClimberConstants.POWER_DOWN);
    }

    /**
     * Alias for recolher() - Retracts the climber arms downwards.
     */
    public void retract() {
        recolher();
    }

    /**
     * Stops all climber servos by setting their power to 0.0.
     */
    public void stop() {
        leftServol.setPower(0.0);
        rightServol.setPower(0.0);
        leftServor.setPower(0.0);
        rightServor.setPower(0.0);
    }

    /**
     * Proportional manual control for climber servos from joystick input.
     *
     * @param potencia Desired power in range [-1.0, 1.0].
     */
    public void setPotenciaManual(double potencia) {
        double safePower = Math.max(-1.0, Math.min(1.0, potencia));

        leftServol.setPower(safePower);
        rightServol.setPower(safePower);
        leftServor.setPower(safePower);
        rightServor.setPower(safePower);
    }

    /**
     * Alias for setPotenciaManual() - Sets manual power on all climber servos.
     *
     * @param power Desired power in range [-1.0, 1.0].
     */
    public void setManualPower(double power) {
        setPotenciaManual(power);
    }
}