package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.tka.robotics.opmode.RedBlueAutonomous;
import org.tka.robotics.opmode.RedBlueOpMode;
import org.tka.robotics.opmode.TeamColor;
import org.tka.robotics.utils.hardware.MainBotHardware;

/**
 * Created by Joshua on 10/1/2016.
 */

@RedBlueAutonomous(name = "Updated Double Beacon Sideways")
public class AutonomousDoubleBeaconSideways2RedBlue extends RedBlueOpMode {
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
        robotHardware.getColorSensor().enableLed(false);

//        double lowLight = 0.28;
//        double highLight = 0.48;
//        double average = (lowLight + highLight) / 2;

        robotHardware.getUtilities().resetDriveMotors();

        waitForStart();

        //heading = -robotHardware.getGyroSensor().getIntegratedZValue();
        telemetry.addData("heading", heading);
        telemetry.update();


        if(teamColor == TeamColor.BLUE)
            robotHardware.getUtilities().navigateToBeaconBlue();

        if(teamColor == TeamColor.RED)
            robotHardware.getUtilities().navigateToBeaconRed();


        robotHardware.getUtilities().sideLineFollow();

        robotHardware.stopAllMotors();
        sleep(1000);

        telemetry.addData("red", robotHardware.getColorSensor().red());
        telemetry.addData("blue", robotHardware.getColorSensor().blue());
        telemetry.update();

        //sleep(1000);

        //detect blue

        if(teamColor == TeamColor.BLUE)
            robotHardware.getUtilities().detectBeaconColorAndAdjustBlue();

        if(teamColor == TeamColor.RED)
            robotHardware.getUtilities().detectBeaconColorAndAdjustRed();

        robotHardware.getUtilities().strafe(1000, 0.2);


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


        if(teamColor == TeamColor.BLUE)
            robotHardware.getUtilities().driveForward(-1000, 0.3);

        if(teamColor == TeamColor.RED)
            robotHardware.getUtilities().driveForward(1000, 0.3);


        // UPDATE

        if(teamColor == TeamColor.BLUE) {
            while(robotHardware.getLightSensor().getLightDetected() < 0.4) {
                robotHardware.getFrontLeftMotor().setPower(-0.15);
                robotHardware.getFrontRightMotor().setPower(-0.15);
                robotHardware.getBackLeftMotor().setPower(-0.15);
                robotHardware.getBackRightMotor().setPower(-0.15);
                idle();
            }
        }

        if(teamColor == TeamColor.RED) {
            while(robotHardware.getLightSensor().getLightDetected() < 0.4) {
                robotHardware.getFrontLeftMotor().setPower(0.15);
                robotHardware.getFrontRightMotor().setPower(0.15);
                robotHardware.getBackLeftMotor().setPower(0.15);
                robotHardware.getBackRightMotor().setPower(0.15);
                idle();
            }
        }


        // readjust to the right slightly


        robotHardware.getUtilities().driveForward(100, 0.3);

        robotHardware.getUtilities().sideLineFollow();

        robotHardware.stopAllMotors();
        //sleep(500);

        telemetry.addData("red", robotHardware.getColorSensor().red());
        telemetry.addData("blue", robotHardware.getColorSensor().blue());
        telemetry.update();

        //sleep(1000);

        //detect blue

        if(teamColor == TeamColor.BLUE)
            robotHardware.getUtilities().detectBeaconColorAndAdjustBlue();

        if(teamColor == TeamColor.RED)
            robotHardware.getUtilities().detectBeaconColorAndAdjustRed();

        robotHardware.getUtilities().strafe(1000, 0.2);


        /*
        while(opModeIsActive()) {
            telemetry.addData("light: ", lightSensor.getLightDetected());

            telemetry.update();
            idle();
        }
        */

    }


}

