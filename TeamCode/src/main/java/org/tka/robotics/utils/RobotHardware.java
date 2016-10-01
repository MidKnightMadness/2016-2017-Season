package org.tka.robotics.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.HashMap;
import java.util.Map;

/**
 * Hardware abstraction for interaction with Carnival Bot
 */
public class RobotHardware {

    private final OpMode parent;
    private final HardwareMap hardwareMap;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public RobotHardware(OpMode opMode) {
        this.parent = opMode;
        this.hardwareMap = this.parent.hardwareMap;
        initialize();
    }

    /**
     * Initializes all the hardware and telemetry on the robot
     */
    private void initialize() {
        telemetry();
        // Reset all the motor configurations
        for (Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            m.getValue().resetDeviceConfigurationForOpMode();
        }
        frontLeft = hardwareMap.dcMotor.get("left_front");
        frontRight = hardwareMap.dcMotor.get("right_front");
        backLeft = hardwareMap.dcMotor.get("left_back");
        backRight = hardwareMap.dcMotor.get("right_back");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Gets the front left motor
     *
     * @return A {@link DcMotor} corresponding to the front left motor
     */
    public DcMotor getFrontLeftMotor() {
        return this.frontLeft;
    }

    /**
     * Gets the front right motor
     *
     * @return A {@link DcMotor} corresponding to the front right motor
     */
    public DcMotor getFrontRightMotor() {
        return this.frontRight;
    }

    /**
     * Gets the back left motor
     *
     * @return A {@link DcMotor} corresponding to the back left motor
     */
    public DcMotor getBackLeftMotor() {
        return this.backLeft;
    }

    /**
     * Gets the back right motor
     *
     * @return A {@link DcMotor} corresponding to the back right motor
     */
    public DcMotor getBackRightMotor() {
        return this.backRight;
    }

    /**
     * Stops <b>all</b> motors on the robot
     */
    public void stopAllMotors() {
        for (Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            m.getValue().setPower(0);
        }
    }

    /**
     * Initializes telemetry to be sent to the driver station.
     * <br><br>
     * <b>Displays the following data:</b>
     * <ul>
     * <li>All motor's power</li>
     * <li>All motor's current encoder position</li>
     * <li>All motor's target encoder position</li>
     * </ul>
     */
    public void telemetry() {
        for (final Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            this.parent.telemetry.addData(m.getKey() + " Power", new Func<Double>() {
                @Override
                public Double value() {
                    return m.getValue().getPower();
                }
            });
            this.parent.telemetry.addData(m.getKey() + " Position", new Func<Integer>() {
                @Override
                public Integer value() {
                    return m.getValue().getCurrentPosition();
                }
            });
            this.parent.telemetry.addData(m.getKey() + " Target", new Func<Integer>() {
                @Override
                public Integer value() {
                    return m.getValue().getTargetPosition();
                }
            });
        }
    }

    /**
     * Reset all encoders on the robot. This will wait until the motors
     */
    public void resetAllEncoders(){
        Map<DcMotor, DcMotor.RunMode> prevRunMode = new HashMap<>();
        for(Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()){
            prevRunMode.put(m.getValue(), m.getValue().getMode());
            m.getValue().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        boolean encodersReset = false;
        while(!encodersReset){
            encodersReset = true;
            for(Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()){
                if(Math.abs(m.getValue().getCurrentPosition()) < 10){
                    encodersReset = false;
                }
            }
            Thread.yield();
        }
        for(Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()){
            m.getValue().setMode(prevRunMode.get(m.getValue()));
        }
    }
}
