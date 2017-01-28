package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.tka.robotics.utils.hardware.MainBotHardware;

@Autonomous(name = "Diagonal Gyro Test", group = "Sensor")
@Disabled
public class DiagonalGyroTest extends LinearOpMode {

    MainBotHardware robotHardware;
    ModernRoboticsI2cGyro gyro;
    int heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robotHardware = new MainBotHardware(this);
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

        // insert code here


        robotHardware.getUtilities().gyroReadjust(10, gyro);






        while(opModeIsActive()) {



            idle();
        }

        /*
        sleep(250);
        turnDegrees(0.2, 90);
        sleep(250);
        turnDegrees(0.2, 90);
        sleep(250);
        turnDegrees(0.2, 90);
        */
    }

    public void turnDegrees(double power, double angle) throws InterruptedException {
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

                telemetry.addData("% to Target", percentToTarget);
                telemetry.addData("Heading", heading);
                telemetry.update();
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

                telemetry.addData("% to Target", percentToTarget);
                telemetry.addData("Heading", heading);
                telemetry.update();
                idle();
            }
        }

        robotHardware.stopAllMotors();


    }

    public void turn(double power) {
        robotHardware.getFrontLeftMotor().setPower(power);
        robotHardware.getBackLeftMotor().setPower(power);
        robotHardware.getFrontRightMotor().setPower(-power);
        robotHardware.getBackRightMotor().setPower(-power);
    }

}
