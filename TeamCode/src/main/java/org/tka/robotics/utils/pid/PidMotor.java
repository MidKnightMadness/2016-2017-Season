package org.tka.robotics.utils.pid;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

public class PidMotor {

    private static final float K_P = 0.001F;
    private static final float K_V = 0.001F;
    private static final int ENC_TICKS_PER_REV = 1008;
    private static final double MAX_SPEED = 2.5;
    protected final DcMotor motor;
    private final File logBaseDir = Environment.getExternalStoragePublicDirectory("PID-Logs");
    public boolean debugLogging = false;
    private boolean reversed = false;
    private double expectedEncoder;
    private double actualEncoder;
    private long lastTime;
    private double lastError;
    // Velocity in ticks per nanosecond
    private double velocity;
    private String name;
    private File logFile;
    private PrintWriter writer;
    private long startTime;
    private int runCount;

    public PidMotor(HardwareMap map, String name) {
        this.motor = map.dcMotor.get(name);
        this.name = name;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.startTime = System.currentTimeMillis();
        if (PidUpdater.INSTANCE == null)
            PidUpdater.INSTANCE = new PidUpdater(map);
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
        if (debugLogging) {
            runCount++;
            if (runCount % 10 == 0)
                writer.flush();
            writer.println((System.currentTimeMillis() - startTime) + "," + power + "," + error);
        }
        motor.setPower(power);
    }

    // Velocity in revolutions per second
    public void setVelocity(double velocity) {
        velocity = Range.clip(velocity, -MAX_SPEED, MAX_SPEED);
        this.velocity = (ENC_TICKS_PER_REV * velocity) / 10e2;
        if (reversed)
            this.velocity *= -1;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        reversed = direction == DcMotorSimple.Direction.REVERSE;
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        setVelocity(MAX_SPEED * power);
    }

    public void addTelemetry(OpMode mode) {
        mode.telemetry.addData(this.name + "-power", new Func<Double>() {
            @Override
            public Double value() {
                return motor.getPower();
            }
        });
        mode.telemetry.addData(this.name + "-encoder", new Func<String>() {
            @Override
            public String value() {
                return expectedEncoder + "," + actualEncoder;
            }
        });
        mode.telemetry.addData(this.name + "-velocity", new Func<Double>() {
            @Override
            public Double value() {
                return velocity;
            }
        });
    }

    public void enableDebugLogging() {
        try {
            logFile = new File(logBaseDir, this.name + ".csv");
            logBaseDir.mkdirs();
            if (!logFile.exists())
                logFile.createNewFile();
            else {
                logFile.delete();
                logFile.createNewFile();
            }
            Log.i("PID-Debug", "Logging to " + logFile.getAbsolutePath());
            writer = new PrintWriter(logFile);
            writer.println("time,power,error");
            debugLogging = true;
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void setPosition(int position) {
        this.expectedEncoder = position;
    }

    DcMotor getMotor() {
        return motor;
    }

    public String getName() {
        return name;
    }
}
