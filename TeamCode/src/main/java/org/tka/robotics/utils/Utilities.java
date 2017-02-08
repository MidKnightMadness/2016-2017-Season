package org.tka.robotics.utils;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.tka.robotics.utils.hardware.RobotHardware;

public class Utilities {

    private RobotHardware hardware;
    private OpMode parent;

    private ModernRoboticsI2cGyro gyro;

    public Utilities(OpMode parent, RobotHardware hardware) {
        this.hardware = hardware;
        this.parent = parent;

        if(hardware.getGyroSensor() instanceof ModernRoboticsI2cGyro)
            gyro = (ModernRoboticsI2cGyro) hardware.getGyroSensor();
    }

    public void turnRight(double power) {
        this.hardware.getFrontLeftMotor().setPower(power);
        this.hardware.getBackLeftMotor().setPower(power);
        this.hardware.getFrontRightMotor().setPower(-power);
        this.hardware.getBackRightMotor().setPower(-power);
    }

    public void driveForward(int distance, double power) throws InterruptedException {
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (distance > 0) {
            while (this.hardware.getFrontRightMotor().getCurrentPosition() < distance) {
                setAllMotors(power);
                idle();
            }
        } else {
            while (this.hardware.getFrontRightMotor().getCurrentPosition() > distance) {
                setAllMotors(-power);
                idle();
            }
        }

        this.hardware.stopAllMotors();
    }

    public void setAllMotors(double power) throws InterruptedException {
        this.hardware.getFrontLeftMotor().setPower(power);
        this.hardware.getFrontRightMotor().setPower(power);
        this.hardware.getBackLeftMotor().setPower(power);
        this.hardware.getBackRightMotor().setPower(power);
    }

    public void forwardLeftDiagonal(double distance, double power) throws InterruptedException {
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getBackLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBackLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFrontRightMotor().setPower(power);
        this.hardware.getBackLeftMotor().setPower(power);
        this.hardware.getFrontLeftMotor().setPower(0);
        this.hardware.getBackRightMotor().setPower(0);

        while (this.hardware.getFrontRightMotor().getCurrentPosition() < distance) {
            this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", this.hardware.getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            idle();
        }

        this.hardware.stopAllMotors();
    }

    public void forwardRightDiagonal(double distance, double power) throws InterruptedException {
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFrontRightMotor().setPower(0);
        this.hardware.getBackLeftMotor().setPower(0);
        this.hardware.getFrontLeftMotor().setPower(power);
        this.hardware.getBackRightMotor().setPower(power);

        while (this.hardware.getFrontLeftMotor().getCurrentPosition() < distance) {
            this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", this.hardware.getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            idle();
        }

        this.hardware.stopAllMotors();
    }


    public void backwardLeftDiagonal(double distance, double power) throws InterruptedException {
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFrontRightMotor().setPower(0);
        this.hardware.getBackLeftMotor().setPower(0);
        this.hardware.getFrontLeftMotor().setPower(-power);
        this.hardware.getBackRightMotor().setPower(-power);

        while (this.hardware.getFrontLeftMotor().getCurrentPosition() > -distance) {
            this.parent.telemetry.addData("front left", this.hardware.getFrontLeftMotor().getCurrentPosition());
            this.parent.telemetry.addData("back right", this.hardware.getBackRightMotor().getCurrentPosition());
            this.parent.telemetry.update();
            idle();
        }

        this.hardware.stopAllMotors();
    }

    public void strafe(double distance, double power) throws InterruptedException {
        if (power < 0)
            throw new IllegalStateException("Power must be positive");

        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (distance > 0) {
            while (this.hardware.getFrontLeftMotor().getCurrentPosition() < distance) {
                this.hardware.getFrontLeftMotor().setPower(power);
                this.hardware.getFrontRightMotor().setPower(-power);
                this.hardware.getBackLeftMotor().setPower(-power);
                this.hardware.getBackRightMotor().setPower(power);
                this.parent.telemetry.addData("front left", this.hardware.getFrontLeftMotor().getCurrentPosition());
                this.parent.telemetry.update();
                idle();
            }
        } else {
            while (this.hardware.getFrontLeftMotor().getCurrentPosition() > distance) {
                this.hardware.getFrontLeftMotor().setPower(-power);
                this.hardware.getFrontRightMotor().setPower(power);
                this.hardware.getBackLeftMotor().setPower(power);
                this.hardware.getBackRightMotor().setPower(-power);
                this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
                this.parent.telemetry.update();
                idle();
            }
        }

        this.hardware.stopAllMotors();
    }

    public void setDriveMotorsMode(DcMotor.RunMode runMode) {
        hardware.getFrontLeftMotor().setMode(runMode);
        hardware.getFrontRightMotor().setMode(runMode);
        hardware.getBackLeftMotor().setMode(runMode);
        hardware.getBackRightMotor().setMode(runMode);
    }

    public void resetDriveMotors() {
        setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void navigateToBeaconBlue() throws InterruptedException{
        hardware.getUtilities().backwardLeftDiagonal(6000, 0.8); // doubled


        while(hardware.getLightSensor().getLightDetected() < 0.4) {
            hardware.getFrontLeftMotor().setPower(-0.4);
            hardware.getFrontRightMotor().setPower(0);
            hardware.getBackLeftMotor().setPower(0);
            hardware.getBackRightMotor().setPower(-0.4);
            idle();
        }

        hardware.stopAllMotors();
    }

    public void navigateToBeaconRed() throws InterruptedException{
        hardware.getUtilities().forwardLeftDiagonal(6000, 0.8); // doubled


        while(hardware.getLightSensor().getLightDetected() < 0.4) {
            hardware.getFrontLeftMotor().setPower(0);
            hardware.getFrontRightMotor().setPower(0.4);
            hardware.getBackLeftMotor().setPower(0.4);
            hardware.getBackRightMotor().setPower(0);
            idle();
        }

        hardware.stopAllMotors();
    }



    public void sideLineFollow() throws InterruptedException {
        while(hardware.getUltrasonicSensor().getUltrasonicLevel() > 16) {


            if(hardware.getLightSensor().getLightDetected() < 0.4) {
                hardware.getFrontLeftMotor().setPower(-0.15+0.04);
                hardware.getFrontRightMotor().setPower(0.15);
                hardware.getBackLeftMotor().setPower(0.15+0.04);
                hardware.getBackRightMotor().setPower(-0.15);
            } else {
                hardware.getFrontLeftMotor().setPower(-0.15-0.04);
                hardware.getFrontRightMotor().setPower(0.15);
                hardware.getBackLeftMotor().setPower(0.15-0.04);
                hardware.getBackRightMotor().setPower(-0.15);
            }

            this.parent.telemetry.addData("light", hardware.getLightSensor().getLightDetected());
            this.parent.telemetry.addData("red", hardware.getColorSensor().red());
            this.parent.telemetry.addData("blue", hardware.getColorSensor().blue());
            this.parent.telemetry.update();

            idle();
        }

        hardware.stopAllMotors();
    }
    
    public void idle() throws InterruptedException {
        if(parent instanceof LinearOpMode) {
            ((LinearOpMode) parent).idle();
        }
    }

    public void detectBeaconColorAndAdjustBlue() throws InterruptedException {
        if(hardware.getColorSensor().blue() > hardware.getColorSensor().red()) {
            //go backward

            parent.telemetry.addData("", "blue > red");
            parent.telemetry.update();

            driveForward(-250, 0.4);
        }
        else {
            //go forward

            parent.telemetry.addData("", "red > blue");
            parent.telemetry.update();

            driveForward(250, 0.4);

        }
    }

    public void detectBeaconColorAndAdjustRed() throws InterruptedException {
        if(hardware.getColorSensor().blue() > hardware.getColorSensor().red()) {
            //go forward

            parent.telemetry.addData("", "blue > red");
            parent.telemetry.update();
            //sleep(500);

            driveForward(250, 0.4);



        }
        else {
            //go backward

            parent.telemetry.addData("", "red > blue");
            parent.telemetry.update();
            //sleep(500);

            driveForward(-250, 0.4);

        }
    }

    public void gyroReadjust(float target, ModernRoboticsI2cGyro gyroSensor) {
        while(Math.abs(target - gyroSensor.getIntegratedZValue()) > 2) {
            turnRight(0.1);
            this.parent.telemetry.addData("target", target);
            this.parent.telemetry.addData("gyroSensor", gyroSensor.getIntegratedZValue());
        }
    }

    public void turnDegrees(double power, double angle) throws InterruptedException {
        if(power < 0)
            throw new IllegalStateException("Power must be positive");

        double heading = -gyro.getIntegratedZValue();

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
                idle();
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
                idle();
            }
        }

        hardware.stopAllMotors();


    }

    public void turnDegreesSmall(double power, double angle) throws InterruptedException {
        if(power < 0)
            throw new IllegalStateException("Power must be positive");

        double heading = -gyro.getIntegratedZValue();

        double initialHeading = heading;
        double targetAngle = heading + angle;
        double percentToTarget = 0;


        if(angle > 0) {
            turn(power);

            while(heading < targetAngle) {
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
                idle();
            }

        }
        else {
            turn(-power);

            while(heading > targetAngle) {
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
                idle();
            }
        }

        hardware.stopAllMotors();


    }

    public void turn(double power) {
        hardware.getFrontLeftMotor().setPower(power);
        hardware.getBackLeftMotor().setPower(power);
        hardware.getFrontRightMotor().setPower(-power);
        hardware.getBackRightMotor().setPower(-power);
    }
}
