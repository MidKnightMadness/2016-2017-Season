package org.tka.robotics.utils;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
    private double heading;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public RobotHardware(OpMode opMode) {
        this.parent = opMode;
        this.hardwareMap = this.parent.hardwareMap;
        initialize();
    }
    
    ModernRoboticsI2cGyro gyro;

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

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void turnDegrees(double angle, double power) throws InterruptedException {
        if(power < 0)
            throw new IllegalStateException("Power must be positive");

        double initialHeading = heading;
        double targetAngle = heading + angle;
        double percentToTarget = 0;
        int offset = 10; //was 12


        if(angle > 0) {
            turn(power);

            while(heading < targetAngle - offset) {
                heading = -gyro.getIntegratedZValue();
                percentToTarget = (heading - initialHeading)/(targetAngle - initialHeading) * 100;

                /*
                if(percentToTarget > 75 && power >= 0.20) {
                    turn(power * 0.75);
                }
                */

                this.parent.telemetry.addData("% to Target", percentToTarget);
                this.parent.telemetry.addData("Heading", heading);
                this.parent.telemetry.update();
                Thread.yield();
            }

        }
        else {
            turn(-power);

            while(heading > targetAngle + offset) {
                heading = -gyro.getIntegratedZValue();
                percentToTarget = (heading - initialHeading)/(targetAngle - initialHeading) * 100;

                /*
                if(percentToTarget > 75 && power >= 0.20) {
                    turn(-power * 0.75);
                }
                */

                this.parent.telemetry.addData("% to Target", percentToTarget);
                this.parent.telemetry.addData("Heading", heading);
                this.parent.telemetry.update();
                Thread.yield();
            }
        }

        stopAllMotors();


    }

    public void turn(double power) {
        getFrontLeftMotor().setPower(power);
        getBackLeftMotor().setPower(power);
        getFrontRightMotor().setPower(-power);
        getBackRightMotor().setPower(-power);
    }

    public void driveForward(int distance, double power) throws InterruptedException {
        getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(distance > 0) {
            while(getFrontRightMotor().getCurrentPosition() < distance) {
                setAllMotors(power);
            }
        }
        else {
            while(getFrontRightMotor().getCurrentPosition() > distance) {
                setAllMotors(-power);
            }
        }



        stopAllMotors();
    }

    public void setAllMotors(double power) throws InterruptedException {
        getFrontLeftMotor().setPower(power);
        getFrontRightMotor().setPower(power);
        getBackLeftMotor().setPower(power);
        getBackRightMotor().setPower(power);
        Thread.yield();
    }

    public void forwardLeftDiagonal(double distance, double power) throws InterruptedException{
        getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getBackLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getBackLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getFrontRightMotor().setPower(power);
        getBackLeftMotor().setPower(power);
        getFrontLeftMotor().setPower(0);
        getBackRightMotor().setPower(0);

        while(getFrontRightMotor().getCurrentPosition() < distance) {
            this.parent.telemetry.addData("front right", getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            Thread.yield();
        }

        stopAllMotors();
    }
}
