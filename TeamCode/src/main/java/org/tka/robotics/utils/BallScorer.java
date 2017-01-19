package org.tka.robotics.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class BallScorer implements Runnable {


    private final DcMotor motor;
    private final CRServo servo;
    private final TouchSensor touchSensor;
    private final OpMode opMode;

    private State state = State.RETRACTING;
    private boolean running = true;

    public BallScorer(OpMode opMode, DcMotor motor, CRServo servo, TouchSensor touchSensor) {
        this.motor = motor;
        this.servo = servo;
        this.touchSensor = touchSensor;
        this.opMode = opMode;
    }

    public DcMotor getMotor() {
        return motor;
    }

    public CRServo getServo() {
        return servo;
    }

    public TouchSensor getTouchSensor() {
        return touchSensor;
    }

    public void LAUNCHDATHING(){
        state = State.FIRING;
    }

    @Override
    public void run() {
        while (running) {
            switch (state) {
                case FIRING:
                    if (!touchSensor.isPressed()) {
                        servo.setPower(0);
                        state = State.RETRACTING;
                    } else {
                        servo.setPower(1);
                    }
                    break;
                case RETRACTING:
                    if (!touchSensor.isPressed()) {
                        motor.setPower(1);
                    } else {
                        motor.setPower(0);
                        state = State.EXTENDING;
                    }
                    break;
                case EXTENDING:
                    if (motor.getCurrentPosition() < 10) {
                        state = State.WAITING;
                        motor.setPower(0);
                    } else {
                        motor.setPower(-1);
                    }
                    break;
            }
            Thread.yield();
        }
    }

    public State getState() {
        return state;
    }

    public void stop() {
        this.running = false;
    }

    public enum State {
        WAITING,
        FIRING,
        RETRACTING,
        EXTENDING;
    }
}
