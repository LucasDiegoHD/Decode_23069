package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo; // <-- Classe Nova!
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClimberSubsystem extends SubsystemBase {

    private final CRServo leftServol;
    private final CRServo leftServor;
    private final CRServo rightServor;
    private final CRServo rightServol;

    public ClimberSubsystem(HardwareMap hardwareMap) {
        leftServol = hardwareMap.get(CRServo.class, ClimberConstants.LEFT_SERVOL_NAME);
        leftServor = hardwareMap.get(CRServo.class, ClimberConstants.LEFT_SERVOR_NAME);
        rightServor = hardwareMap.get(CRServo.class, ClimberConstants.RIGHT_SERVOR_NAME);
        rightServol = hardwareMap.get(CRServo.class, ClimberConstants.RIGHT_SERVOL_NAME);

        // A direção do CRServo usa DcMotorSimple
        leftServol.setDirection(DcMotorSimple.Direction.FORWARD);
        rightServol.setDirection(DcMotorSimple.Direction.FORWARD);
        leftServor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightServor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Medida de segurança: Garante que o motor nasce desligado (força 0)
        stop();
    }

    /**
     * Manda os servos girarem para cima (esticar o braço).
     */
    public void esticar() {
        leftServol.setPower(-ClimberConstants.POWER_UP);
        rightServol.setPower(-ClimberConstants.POWER_UP);
        leftServor.setPower(-ClimberConstants.POWER_UP);
        rightServor.setPower(-ClimberConstants.POWER_UP);
    }

    /**
     * Manda os servos girarem para baixo (recolher/puxar o robô).
     */
    public void recolher() {
        leftServol.setPower(-ClimberConstants.POWER_DOWN);
        rightServol.setPower(-ClimberConstants.POWER_DOWN);
        leftServor.setPower(-ClimberConstants.POWER_DOWN);
        rightServor.setPower(-ClimberConstants.POWER_DOWN);
    }

    public void stop(){
        leftServol.setPower(0.0);
        rightServol.setPower(0.0);
        leftServor.setPower(0.0);
        rightServor.setPower(0.0);
    }

    /**
     * Controle manual por joystick (recebe valores de -1.0 a 1.0)
     */
    public void setPotenciaManual(double potencia) {
        // Limita a potência por segurança para não mandar mais de 1.0 ou menos de -1.0
        double potSegura = Math.max(-1.0, Math.min(1.0, potencia));

        leftServol.setPower(potSegura);
        rightServol.setPower(potSegura);
        leftServor.setPower(potSegura);
        rightServor.setPower(potSegura);
    }
}