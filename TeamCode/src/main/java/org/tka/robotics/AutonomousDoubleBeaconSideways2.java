package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.hardware.MainBotHardware;
import org.tka.robotics.utils.hardware.RobotHardware;
import org.tka.robotics.utils.hardware.SoftwareBotHardware;

/**
 * Created by Joshua on 10/1/2016.
 */

@Autonomous(name = "Updated Double Beacon Sideways")
public class AutonomousDoubleBeaconSideways2 extends LinearOpMode {
    MainBotHardware robotHardware;
    //LightSensor lightSensor;
    //ColorSensor colorSensor;
    //ModernRoboticsI2cGyro gyro;
    int heading = 0;
    double offset = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new MainBotHardware(this);
        //lightSensor = robotHardware.getLightSensor();
        //colorSensor = hardwareMap.colorSensor.get("color_sensor");
        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        //robotHardware.getGyroSensor().calibrate();
        //heading = -robotHardware.getGyroSensor().getIntegratedZValue();

        //gyro declared in SoftwareBotHardware

        robotHardware.getLightSensor().enableLed(true);

//        double lowLight = 0.28;
//        double highLight = 0.48;
//        double average = (lowLight + highLight) / 2;

        robotHardware.getUtilities().resetDriveMotors();

        waitForStart();

        //heading = -robotHardware.getGyroSensor().getIntegratedZValue();
        telemetry.addData("heading", heading);
        telemetry.update();


        robotHardware.getUtilities().navigateToBeacon();
        //sleep(500);

        robotHardware.getUtilities().sideLineFollow();

        robotHardware.stopAllMotors();
        sleep(1000);

        telemetry.addData("red", robotHardware.getColorSensor().red());
        telemetry.addData("blue", robotHardware.getColorSensor().blue());
        telemetry.update();

        //sleep(1000);

        //detect blue

        if(robotHardware.getColorSensor().blue() > robotHardware.getColorSensor().red()) {
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

            robotHardware.getUtilities().driveForward(400, 0.2);

            // FIX //
            // robotHardware.driveForward(400, 0.2);
        }

        robotHardware.getUtilities().strafe(-400, 0.2);



        robotHardware.getUtilities().strafe(1000, 0.2);

        sleep(10000);


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

        robotHardware.getUtilities().driveForward(-1000, 0.4);


        // UPDATE

        while(robotHardware.getLightSensor().getLightDetected() < 0.4) {
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
            idle();
        }

        while((robotHardware.getColorSensor().red() <= 1) && robotHardware.getColorSensor().blue() <= 1) {

            if(robotHardware.getLightSensor().getLightDetected() < 0.4) {
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

            telemetry.addData("red", robotHardware.getColorSensor().red());
            telemetry.addData("blue", robotHardware.getColorSensor().blue());
            telemetry.update();

            idle();
        }


        robotHardware.stopAllMotors();
        //sleep(500);

        telemetry.addData("red", robotHardware.getColorSensor().red());
        telemetry.addData("blue", robotHardware.getColorSensor().blue());
        telemetry.update();

        //sleep(1000);

        //detect blue

        if(robotHardware.getColorSensor().blue() > robotHardware.getColorSensor().red()) {
            //go forward

            telemetry.addData("", "blue > red");
            telemetry.update();
            //sleep(500);


            robotHardware.getUtilities().driveForward(400, 0.2);
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
                idle();
            }
            robotHardware.stopAllMotors();

            robotHardware.getUtilities().driveForward(400, 0.2);
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

