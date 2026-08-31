package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Limita a taxa de variação de um valor no tempo (Rampa de Aceleração).
 * Essencial para evitar wheel slip (derrapagem) no robô.
 */
public class SlewRateLimiter {
    private double rateLimit;
    private double prevVal;
    private final ElapsedTime timer;

    /**
     * @param rateLimit Quanto o valor pode mudar por segundo.
     * Ex: 2.0 significa que vai de 0 a 1.0 em 0.5 segundos.
     */
    public SlewRateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
        this.prevVal = 0.0;
        this.timer = new ElapsedTime();
    }

    public void setRateLimit(double rateLimit) {
        this.rateLimit = rateLimit;
    }

    public double calculate(double input) {
        double dt = timer.seconds();
        timer.reset();

        // Calcula o máximo que a potência pode subir ou descer neste milissegundo
        double maxChange = rateLimit * dt;

        // Limita o degrau de aceleração
        if (input > prevVal + maxChange) {
            prevVal += maxChange;
        } else if (input < prevVal - maxChange) {
            prevVal -= maxChange;
        } else {
            prevVal = input;
        }

        return prevVal;
    }

    public void reset(double value) {
        prevVal = value;
        timer.reset();
    }
}