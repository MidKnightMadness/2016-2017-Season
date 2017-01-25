package org.tka.robotics.utils;

import android.widget.TextView;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.lang.reflect.Field;

import static org.tka.robotics.utils.BallScorer.State.*;

public class BallScorer implements Runnable {

    private static final int ENCODER_EXTEND_POSITION = -4342;


    private final DcMotor motor;
    private final Servo servo;
    private final TouchSensor touchSensor;
    private final OpMode opMode;

    private State state = HOMING;

    private long waitUntil = -1;
    private boolean running = true;

    private int targetPos = 0;

    private Field updateUIField, robotState;

    public BallScorer(OpMode opMode, DcMotor motor, Servo servo, TouchSensor touchSensor) {
        this.motor = motor;
        this.servo = servo;
        this.touchSensor = touchSensor;
        this.opMode = opMode;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public Servo getServo() {
        return servo;
    }

    public TouchSensor getTouchSensor() {
        return touchSensor;
    }

    public void launch() {
        if (state == WAITING)
            state = FIRING;
    }

    public void home() {
        state = HOMING;
    }

    @Override
    public void run() {
        while (running) {
            switch (state) {
                case HOMING:
                    if (!touchSensor.isPressed()) {
                        motor.setPower(1);
                        servo.setPosition(0);
                    } else {
                        motor.setPower(0);
                        DcMotor.RunMode lastMode = motor.getMode();
                        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motor.setMode(lastMode);
                        state = RETRACTING;
                    }
                    break;
                case FIRING:
                    if (!touchSensor.isPressed()) {
                        servo.setPosition(0);
                        state = State.WAITING_FOR_FIRE;
                        waitUntil = System.currentTimeMillis() + 250;
                    } else {
                        servo.setPosition(0.5);
                    }
                    break;
                case WAITING_FOR_FIRE:
                    if (System.currentTimeMillis() > waitUntil) {
                        state = RETRACTING;
                    }
                    break;
                case RETRACTING:
                    if (!touchSensor.isPressed()) {
                        motor.setPower(1);
                    } else {
                        motor.setPower(0);
                        state = State.RETRACTING_PHASE_2;
                        targetPos = motor.getCurrentPosition() + 200;
                    }
                    break;
                case RETRACTING_PHASE_2:
                    if (motor.getCurrentPosition() >= targetPos) {
                        motor.setPower(0);
                        state = EXTENDING;
                    } else {
                        motor.setPower(1);
                    }
                    break;
                case EXTENDING:
                    if (motor.getCurrentPosition() < ENCODER_EXTEND_POSITION) {
                        if (touchSensor.isPressed())
                            state = WAITING;
                        else
                            state = RETRACTING;
                        motor.setPower(0);
                    } else {
                        motor.setPower(-1);
                    }
                    break;
            }
            Thread.yield();
            if (runningOpMode().equalsIgnoreCase("Stop Robot")) {
                this.running = false;
            }
        }
    }

    public State getState() {
        return state;
    }

    public void stop() {
        this.running = false;
    }

    public enum State {
        HOMING,
        WAITING,
        FIRING,
        WAITING_FOR_FIRE,
        RETRACTING,
        RETRACTING_PHASE_2,
        EXTENDING
    }

    /**
     * <b>WARNING!!! ADVANCED CODING AHEAD!</b>
     * <br/>
     * Get the current op mode that is running
     *
     * @return The op mode that is running
     */
    private String runningOpMode() {
        try {
            if (updateUIField == null) {
                updateUIField = FtcRobotControllerActivity.class.getDeclaredField("updateUI");
                updateUIField.setAccessible(true);
            }
            UpdateUI ui = (UpdateUI) updateUIField.get(opMode.hardwareMap.appContext);
            if (robotState == null) {
                robotState = UpdateUI.class.getDeclaredField("textOpMode");
                robotState.setAccessible(true);
            }
            return ((TextView) robotState.get(ui)).getText().toString().substring(9);
        } catch (IllegalAccessException | NoSuchFieldException e) {
            e.printStackTrace();
        }
        return "Stop Robot";
    }
}
