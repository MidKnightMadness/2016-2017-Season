package org.tka.robotics.utils.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.HashMap;
import java.util.Map;

/**
 * Hardware abstraction for interaction with Carnival Bot
 */
public class SoftwareBotHardware extends RobotHardware{
    private double heading;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public SoftwareBotHardware(OpMode opMode) {
        super(opMode);
    }

    ModernRoboticsI2cGyro gyro;

    /**
     * Initializes all the hardware and telemetry on the robot
     */
    @Override
    public void initialize() {
        telemetry();
        // Reset all the motor configurations
        for (Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            m.getValue().resetDeviceConfigurationForOpMode();
            m.getValue().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    @Override
    public DcMotor getFrontLeftMotor() {
        return this.frontLeft;
    }

    /**
     * Gets the front right motor
     *
     * @return A {@link DcMotor} corresponding to the front right motor
     */
    @Override
    public DcMotor getFrontRightMotor() {
        return this.frontRight;
    }

    /**
     * Gets the back left motor
     *
     * @return A {@link DcMotor} corresponding to the back left motor
     */
    @Override
    public DcMotor getBackLeftMotor() {
        return this.backLeft;
    }

    /**
     * Gets the back right motor
     *
     * @return A {@link DcMotor} corresponding to the back right motor
     */
    @Override
    public DcMotor getBackRightMotor() {
        return this.backRight;
    }

    /**
     * Stops <b>all</b> motors on the robot
     */
    @Override
    public void stopAllMotors() {
        for (Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            m.getValue().setPower(0);
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

    public void forwardRightDiagonal(double distance, double power) throws InterruptedException{
        getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getFrontRightMotor().setPower(0);
        getBackLeftMotor().setPower(0);
        getFrontLeftMotor().setPower(power);
        getBackRightMotor().setPower(power);

        while(getFrontLeftMotor().getCurrentPosition() < distance) {
            this.parent.telemetry.addData("front right", getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            Thread.yield();
        }

        stopAllMotors();
    }



    public void backwardLeftDiagonal(double distance, double power) throws InterruptedException{
        getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getFrontRightMotor().setPower(0);
        getBackLeftMotor().setPower(0);
        getFrontLeftMotor().setPower(-power);
        getBackRightMotor().setPower(-power);

        while(getFrontLeftMotor().getCurrentPosition() > -distance) {
            this.parent.telemetry.addData("front right", getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            Thread.yield();
        }

        stopAllMotors();
    }

    public void strafe(double distance, double power) throws InterruptedException{
        if(power < 0)
            throw new IllegalStateException("Power must be positive");

        getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if(distance > 0) {


            while (getFrontLeftMotor().getCurrentPosition() < distance) {
                getFrontLeftMotor().setPower(power);
                getFrontRightMotor().setPower(-power);
                getBackLeftMotor().setPower(-power);
                getBackRightMotor().setPower(power);
                this.parent.telemetry.addData("front left", getFrontLeftMotor().getCurrentPosition());
                this.parent.telemetry.update();
                Thread.yield();
            }
        } else {

            while (getFrontLeftMotor().getCurrentPosition() > distance) {
                getFrontLeftMotor().setPower(-power);
                getFrontRightMotor().setPower(power);
                getBackLeftMotor().setPower(power);
                getBackRightMotor().setPower(-power);
                this.parent.telemetry.addData("front right", getFrontRightMotor().getCurrentPosition());
                this.parent.telemetry.update();
                Thread.yield();
            }
        }

        stopAllMotors();

    }
}
