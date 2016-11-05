package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.hardware.SoftwareBotHardware;

/**
 * Created by Joshua on 10/1/2016.
 */

@Autonomous(name = "Line Follow Color Test")
public class AutonomousLineFollowColorDetectTest extends LinearOpMode {
    SoftwareBotHardware robotHardware;
    LightSensor lightSensor;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new SoftwareBotHardware(this);
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        lightSensor.enableLed(true);

        double lowLight = 0.28;
        double highLight = 0.48;
        double average = (lowLight + highLight) / 2;
        double residual;
        double correctedResidual;

        double leftPower;
        double rightPower;

        waitForStart();

        driveForward(1000, 0.3);
        sleep(500);

        while(lightSensor.getLightDetected() < 0.3) {
            setAllMotors(0.25);
            idle();
        }
        robotHardware.stopAllMotors();
        sleep(500);

        while((colorSensor.red() <= 1) && colorSensor.blue() <= 1) {

            /*
            double offset = 0.05;
            if(lightSensor.getLightDetected() < 0.3) {
                robotHardware.getFrontLeftMotor().setPower(-0.25 + offset);
                robotHardware.getFrontRightMotor().setPower(0.25 + offset);
                robotHardware.getBackLeftMotor().setPower(0.25 + offset);
                robotHardware.getBackRightMotor().setPower(-0.25 + offset);
            } else {
                robotHardware.getFrontLeftMotor().setPower(0.25 + offset);
                robotHardware.getFrontRightMotor().setPower(-0.25 + offset);
                robotHardware.getBackLeftMotor().setPower(-0.25 + offset);
                robotHardware.getBackRightMotor().setPower(0.25 + offset);
            }
            */
            /*
            residual = lightSensor.getLightDetected() - average;
            correctedResidual = residual * 0.3;

            leftPower = 0.05 + correctedResidual;
            rightPower = 0.05 - correctedResidual;

            robotHardware.getFrontLeftMotor().setPower(leftPower);
            robotHardware.getFrontRightMotor().setPower(rightPower);
            robotHardware.getBackLeftMotor().setPower(leftPower);
            robotHardware.getBackRightMotor().setPower(rightPower);

            telemetry.addData("Residual", residual);
            telemetry.addData("C Residudal", correctedResidual);
            telemetry.addData("leftPower", leftPower);
            telemetry.addData("rightPower", rightPower);
            telemetry.update();
            */


            if(lightSensor.getLightDetected() < 0.35) {
                robotHardware.getFrontLeftMotor().setPower(0);
                robotHardware.getFrontRightMotor().setPower(0.15);
                robotHardware.getBackLeftMotor().setPower(0);
                robotHardware.getBackRightMotor().setPower(0.15);
            } else {
                robotHardware.getFrontLeftMotor().setPower(0.15);
                robotHardware.getFrontRightMotor().setPower(0);
                robotHardware.getBackLeftMotor().setPower(0.15);
                robotHardware.getBackRightMotor().setPower(0);
            }

            telemetry.addData("red", colorSensor.red());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.update();

            idle();
        }

        robotHardware.stopAllMotors();
        sleep(250);

        //detect blue

        if(colorSensor.red() > colorSensor.blue()) {
            //go forward


            driveForward(200, 0.2);
        }
        else if(colorSensor.blue() > colorSensor.red()) {
            //go right

            robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while(robotHardware.getFrontRightMotor().getCurrentPosition() > -250) {
                robotHardware.getFrontLeftMotor().setPower(0.20);
                robotHardware.getFrontRightMotor().setPower(-0.20);
                robotHardware.getBackLeftMotor().setPower(-0.20);
                robotHardware.getBackRightMotor().setPower(0.20);
                telemetry.addData("frontRight", robotHardware.getFrontRightMotor().getCurrentPosition());
            }
            robotHardware.stopAllMotors();

            driveForward(200, 0.2);

        } else {

        }

        /*
        while(opModeIsActive()) {
            telemetry.addData("light: ", lightSensor.getLightDetected());

            telemetry.update();
            idle();
        }
        */

    }

    private void driveForward(int distance, double power) throws InterruptedException {
        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(robotHardware.getFrontRightMotor().getCurrentPosition() < distance) {
            setAllMotors(power);
        }

        robotHardware.stopAllMotors();
    }

    private void setAllMotors(double power) throws InterruptedException {
        robotHardware.getFrontLeftMotor().setPower(power);
        robotHardware.getFrontRightMotor().setPower(power);
        robotHardware.getBackLeftMotor().setPower(power);
        robotHardware.getBackRightMotor().setPower(power);
        idle();
    }
}

