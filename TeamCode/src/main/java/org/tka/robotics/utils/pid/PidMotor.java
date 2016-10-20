package org.tka.robotics.utils.pid;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class PidMotor extends Thread {

    private static final float K_P = 0.001F;
    private static final float K_V = 0.001F;
    private static final int ENC_TICKS_PER_REV = 1008;
    private static final double MAX_SPEED = 2.1;
    protected final DcMotor motor;
    public boolean running = true;
    private double expectedEncoder;
    private double actualEncoder;
    private long lastTime;
    private double lastError;
    // Velocity in ticks per nanosecond
    private double velocity;

    public PidMotor(DcMotor motor) {
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (PidUpdater.INSTANCE == null)
            PidUpdater.INSTANCE = new PidUpdater();
        PidUpdater.INSTANCE.registerMotor(this);
    }

    protected void init() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lastError = 0;
        this.expectedEncoder = 0;
        this.actualEncoder = 0;
        this.lastTime = System.currentTimeMillis() - 1;
    }

    protected void update() {
        // --- Calculate E Prime ---
        long currentTime = System.currentTimeMillis();
        long deltaTime = currentTime - this.lastTime;
        this.expectedEncoder += velocity * deltaTime;

        actualEncoder = this.motor.getCurrentPosition();
        double error = expectedEncoder - actualEncoder;
        double deltaError = error - this.lastError;

        double ePrime = deltaError / deltaTime;
        // --- End Calculate E Prime ---

        this.lastError = error;
        this.lastTime = currentTime;

        // -- Do Actual PID Calculation -- Pwr = Kp(e) - Kv(e')
        double power = Range.clip(K_P * error - K_V * ePrime, -1, 1);
        // -- End Actual PID Calculation --

        motor.setPower(power);
    }

    // Velocity in revolutions per second
    public void setVelocity(double velocity) {
        velocity = Range.clip(velocity, -MAX_SPEED, MAX_SPEED);
        this.velocity = (ENC_TICKS_PER_REV * velocity) / 10e2;
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        setVelocity(MAX_SPEED * power);
    }
}
