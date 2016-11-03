package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.RobotHardware;

/**
 * Created by Joshua on 10/1/2016.
 */

@Autonomous(name = "Double Beacon Sideways")
public class AutonomousDoubleBeaconSideways extends LinearOpMode {
    RobotHardware robotHardware;
    LightSensor lightSensor;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;
    int heading = 0;
    double offset = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");


        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();
        heading = -gyro.getIntegratedZValue();

        //gyro declared in RobotHardware

        lightSensor.enableLed(true);

//        double lowLight = 0.28;
//        double highLight = 0.48;
//        double average = (lowLight + highLight) / 2;

        robotHardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotHardware.getBackLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getBackLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotHardware.getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        /*
        robotHardware.setAllMotors(0.2);
        sleep(750);
        robotHardware.stopAllMotors();
        sleep(250);
        robotHardware.setAllMotors(0.4);
        sleep(750);
        robotHardware.stopAllMotors();
        sleep(250);
        robotHardware.setAllMotors(0.7);
        sleep(750);
        robotHardware.stopAllMotors();
        sleep(250);
        robotHardware.setAllMotors(-0.3);
        sleep(750);
        robotHardware.stopAllMotors();
        */

        heading = -gyro.getIntegratedZValue();
        telemetry.addData("heading", heading);
        telemetry.update();


        robotHardware.forwardRightDiagonal(3000, 0.4);
        sleep(250);


        while(lightSensor.getLightDetected() < 0.4) {
            robotHardware.getBackLeftMotor().setPower(0);
            robotHardware.getFrontRightMotor().setPower(0);
            robotHardware.getFrontLeftMotor().setPower(0.2);
            robotHardware.getBackRightMotor().setPower(0.2);
            idle();
        }
        robotHardware.stopAllMotors();
        //sleep(500);

        while((colorSensor.red() <= 1) && colorSensor.blue() <= 1) {

            if(lightSensor.getLightDetected() < 0.4) {
                robotHardware.getFrontLeftMotor().setPower(0.1);
                robotHardware.getFrontRightMotor().setPower(0.1);
                robotHardware.getBackLeftMotor().setPower(-0.1);
                robotHardware.getBackRightMotor().setPower(0.1);
            } else {
                robotHardware.getFrontLeftMotor().setPower(0.1);
                robotHardware.getFrontRightMotor().setPower(-0.1);
                robotHardware.getBackLeftMotor().setPower(-0.1);
                robotHardware.getBackRightMotor().setPower(-0.1);
            }

            telemetry.addData("red", colorSensor.red());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.update();

            idle();
        }

        robotHardware.stopAllMotors();
        //sleep(500);

        telemetry.addData("red", colorSensor.red());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.update();

        //sleep(1000);

        //detect blue

        if(colorSensor.blue() > colorSensor.red()) {
            //go forward

            telemetry.addData("", "blue > red");
            telemetry.update();
            //sleep(500);


            // FIX //
            // robotHardware.driveForward(400, 0.2);

        }
        else {
            //go right

            telemetry.addData("", "red > blue");
            telemetry.update();
            //sleep(500);

            robotHardware.driveForward(-400, 0.2);

            // FIX //
            // robotHardware.driveForward(400, 0.2);
        }

        robotHardware.strafe(400, 0.2);



        robotHardware.strafe(-1000, 0.2);


        //////////////////
        //  2nd beacon  //
        //////////////////

        /*
        heading = -gyro.getIntegratedZValue();
        telemetry.addData("heading", heading);
        telemetry.update();
        sleep(5000);
        */


        //////////////////////////////////
        // NOT UPDATED BELOW THIS POINT //
        //////////////////////////////////


        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (robotHardware.getFrontRightMotor().getCurrentPosition() < 2000) {
            robotHardware.getFrontLeftMotor().setPower(-0.3);
            robotHardware.getFrontRightMotor().setPower(0.3);
            robotHardware.getBackLeftMotor().setPower(0.3);
            robotHardware.getBackRightMotor().setPower(-0.3);
            telemetry.addData("frontRight", robotHardware.getFrontRightMotor().getCurrentPosition());
            telemetry.update();
        }

        while(lightSensor.getLightDetected() < 0.4) {
            robotHardware.getFrontLeftMotor().setPower(-0.15);
            robotHardware.getFrontRightMotor().setPower(0.15);
            robotHardware.getBackLeftMotor().setPower(0.15);
            robotHardware.getBackRightMotor().setPower(-0.15);
            idle();
        }

        // readjust to the right slightly

        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (robotHardware.getFrontRightMotor().getCurrentPosition() > -100) {
            robotHardware.getFrontLeftMotor().setPower(0.15);
            robotHardware.getFrontRightMotor().setPower(-0.15);
            robotHardware.getBackLeftMotor().setPower(-0.15);
            robotHardware.getBackRightMotor().setPower(0.15);
            telemetry.addData("frontRight", robotHardware.getFrontRightMotor().getCurrentPosition());
            telemetry.update();
        }

        while((colorSensor.red() <= 1) && colorSensor.blue() <= 1) {

            if(lightSensor.getLightDetected() < 0.4) {
                robotHardware.getFrontLeftMotor().setPower(0);
                robotHardware.getFrontRightMotor().setPower(0.2);
                robotHardware.getBackLeftMotor().setPower(0);
                robotHardware.getBackRightMotor().setPower(0.2);
            } else {
                robotHardware.getFrontLeftMotor().setPower(0.2);
                robotHardware.getFrontRightMotor().setPower(0);
                robotHardware.getBackLeftMotor().setPower(0.2);
                robotHardware.getBackRightMotor().setPower(0);
            }

            telemetry.addData("red", colorSensor.red());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.update();

            idle();
        }


        robotHardware.stopAllMotors();
        //sleep(500);

        telemetry.addData("red", colorSensor.red());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.update();

        //sleep(1000);

        //detect blue

        if(colorSensor.blue() > colorSensor.red()) {
            //go forward

            telemetry.addData("", "blue > red");
            telemetry.update();
            //sleep(500);


            robotHardware.driveForward(400, 0.2);
        }
        else {
            //go right

            telemetry.addData("", "red > blue");
            telemetry.update();
            //sleep(500);

            robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (robotHardware.getFrontRightMotor().getCurrentPosition() > -200) {
                robotHardware.getFrontLeftMotor().setPower(0.2);
                robotHardware.getFrontRightMotor().setPower(-0.2);
                robotHardware.getBackLeftMotor().setPower(-0.2);
                robotHardware.getBackRightMotor().setPower(0.2);
                telemetry.addData("frontRight", robotHardware.getFrontRightMotor().getCurrentPosition());
                telemetry.update();
            }
            robotHardware.stopAllMotors();

            robotHardware.driveForward(400, 0.2);
        }


        /*
        while(opModeIsActive()) {
            telemetry.addData("light: ", lightSensor.getLightDetected());

            telemetry.update();
            idle();
        }
        */

    }


}

