package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.RobotHardware;

/**
 * Created by Joshua on 10/1/2016.
 */

@Autonomous(name = "Line Follow Color Detect Gyro Turn Test")
public class AutonomousLineFollowColorDetectGyroTurnTest extends LinearOpMode {
    RobotHardware robotHardware;
    LightSensor lightSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this);
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        lightSensor.enableLed(true);

        waitForStart();

        driveForward(1000, 0.3);
        sleep(500);

        while(lightSensor.getLightDetected() < 0.3) {
            setAllMotors(0.25);
            idle();
        }
        robotHardware.stopAllMotors();
        sleep(500);

        while(true) {

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

            idle();
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

