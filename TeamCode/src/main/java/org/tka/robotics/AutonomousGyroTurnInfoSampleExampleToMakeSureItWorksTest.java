package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.tka.robotics.utils.RobotHardware;

@Autonomous(name = "Gyro Test", group = "Sensor")
public class AutonomousGyroTurnInfoSampleExampleToMakeSureItWorksTest extends LinearOpMode {

    RobotHardware robotHardware;
    ModernRoboticsI2cGyro gyro;
    int heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new RobotHardware(this);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();



        turnDegrees(0.2, 90);
        sleep(250);
        turnDegrees(0.2, 90);
        sleep(250);
        turnDegrees(0.2, 90);
        sleep(250);
        turnDegrees(0.2, 90);
    }

    private void turnDegrees(double power, double angle) throws InterruptedException {
        double targetAngle = heading + angle;

        turn(power);

        while(heading < targetAngle) {
            heading = gyro.getHeading();


            telemetry.addData("0", "Heading %03d", heading);
            telemetry.update();
            idle();
        }
        robotHardware.stopAllMotors();
    }

    private void turn(double power) {
        robotHardware.getFrontLeftMotor().setPower(power);
        robotHardware.getBackLeftMotor().setPower(power);
        robotHardware.getFrontRightMotor().setPower(-power);
        robotHardware.getBackRightMotor().setPower(-power);
    }

}
