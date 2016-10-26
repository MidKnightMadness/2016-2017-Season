package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.RobotHardware;

/**
 * Created by Joshua on 10/1/2016.
 */

@Autonomous(name = "Double Beacon")
public class AutonomousDoubleBeacon1 extends LinearOpMode {
    RobotHardware robotHardware;
    LightSensor lightSensor;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        //gyro declared in RobotHardware

        lightSensor.enableLed(true);

//        double lowLight = 0.28;
//        double highLight = 0.48;
//        double average = (lowLight + highLight) / 2;

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


        robotHardware.forwardLeftDiagonal(2500, 0.4);
        sleep(250);


        while(lightSensor.getLightDetected() < 0.3) {
            robotHardware.getFrontRightMotor().setPower(0.3);
            robotHardware.getBackLeftMotor().setPower(0.3);
            robotHardware.getFrontLeftMotor().setPower(0);
            robotHardware.getBackRightMotor().setPower(0);
            idle();
        }
        robotHardware.stopAllMotors();
        //sleep(500);

        while((colorSensor.red() <= 1) && colorSensor.blue() <= 1) {

            if(lightSensor.getLightDetected() < 0.40) {
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

            while (robotHardware.getFrontRightMotor().getCurrentPosition() > -400) {
                robotHardware.getFrontLeftMotor().setPower(0.20);
                robotHardware.getFrontRightMotor().setPower(-0.20);
                robotHardware.getBackLeftMotor().setPower(-0.20);
                robotHardware.getBackRightMotor().setPower(0.20);
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

