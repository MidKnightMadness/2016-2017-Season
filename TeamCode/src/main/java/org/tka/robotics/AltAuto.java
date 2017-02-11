package org.tka.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.tka.robotics.opmode.RedBlueAutonomous;
import org.tka.robotics.opmode.RedBlueOpMode;
import org.tka.robotics.opmode.TeamColor;
import org.tka.robotics.utils.BallScorer;
import org.tka.robotics.utils.hardware.MainBotHardware;
import org.tka.robotics.utils.vuforia.FtcVuforia;

import java.lang.reflect.Field;
import java.util.Arrays;

@RedBlueAutonomous(name = "Alternate Autonomous")
public class AltAuto extends RedBlueOpMode {

    private static float INITIAL_HEADING;
    int heading = 0;
    private MainBotHardware hardware;
    private int red, green, blue;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new MainBotHardware(this);


        telemetry.log().setCapacity(10);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        hardware.getGyroSensor().calibrate();

        while (hardware.getGyroSensor().isCalibrating()) {
            idle();
        }

        INITIAL_HEADING = -hardware.getGyroSensor().getIntegratedZValue();

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        telemetry.log().add("Initialized and ready!");


        hardware.getBallScorer().start();
        hardware.getBallHolderServo().setPosition(0);

        waitForStart();


        hardware.getUtilities().resetDriveMotors();

        if(teamColor == TeamColor.BLUE) {
            hardware.getUtilities().strafe(2000, 0.8);
            hardware.getUtilities().turnDegrees(0.5, 50);
            hardware.getUtilities().strafe(4250, 0.8);

            launchBall();

            sleep(15);

            hardware.getBallHolderServo().setPosition(0.5);

            sleep(15);

            while(hardware.getBallScorer().getState() != BallScorer.State.WAITING) {
                sleep(10);
            }

            launchBall();
            sleep(15);

            hardware.getUtilities().turnDegrees(0.5, 90);

            hardware.getUtilities().driveForward(2750, 0.5);
        }

        if(teamColor == TeamColor.RED) {
            hardware.getUtilities().strafe(2000, 0.8);
            hardware.getUtilities().turnDegrees(0.5, -50);
            hardware.getUtilities().strafe(4250, 0.8);


            launchBall();

            sleep(15);

            hardware.getBallHolderServo().setPosition(0.5);

            sleep(15);

            while(hardware.getBallScorer().getState() != BallScorer.State.WAITING) {
                sleep(10);
            }

            launchBall();
            sleep(15);

            hardware.getUtilities().turnDegrees(0.5, 90);

            hardware.getUtilities().driveForward(2750, 0.5);
        }



        while (opModeIsActive())
            idle();
    }

    private void readjustOrientation(float initialHeading) throws InterruptedException {
        float readjustDegrees = initialHeading - (-hardware.getGyroSensor().getIntegratedZValue());

        telemetry.addData("readjustDegrees", readjustDegrees);
        telemetry.update();

        hardware.getUtilities().turnDegreesSmall(0.2, readjustDegrees);
    }

    private void launchBall() throws InterruptedException {
        if (teamColor == TeamColor.RED)
            readjustOrientation(INITIAL_HEADING);
        while (hardware.getBallScorer().getState() != BallScorer.State.WAITING) {
            idle();
        }

        hardware.getBallScorer().launch();
    }


    private void driveSideways(double power) {
        hardware.getFrontLeftMotor().setPower(power);
        hardware.getBackLeftMotor().setPower(-power);
        hardware.getFrontRightMotor().setPower(-power);
        hardware.getBackRightMotor().setPower(power);
    }

    private void setAllMotorModes(DcMotor.RunMode mode) {
        try {
            for (Field f : hardware.getClass().getDeclaredFields()) {
                if (f.getType() == DcMotor.class) {
                    f.setAccessible(true);
                    DcMotor m = (DcMotor) f.get(hardware);
                    m.setMode(mode);
                }
            }
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    private void driveForward(float motorPower) {
        hardware.getFrontLeftMotor().setPower(motorPower);
        hardware.getFrontRightMotor().setPower(motorPower);
        hardware.getBackLeftMotor().setPower(motorPower);
        hardware.getBackRightMotor().setPower(motorPower);
    }

    private void driveBackLeftDiagonal(float motorPower) throws InterruptedException {
        hardware.getFrontLeftMotor().setPower(-motorPower);
        hardware.getFrontRightMotor().setPower(0);
        hardware.getBackLeftMotor().setPower(0);
        hardware.getBackRightMotor().setPower(-motorPower);
        idle();
    }

    private void driveForwardLeftDiagonal(float motorPower) throws InterruptedException {
        hardware.getFrontLeftMotor().setPower(0);
        hardware.getFrontRightMotor().setPower(motorPower);
        hardware.getBackLeftMotor().setPower(motorPower);
        hardware.getBackRightMotor().setPower(0);
        idle();
    }
}