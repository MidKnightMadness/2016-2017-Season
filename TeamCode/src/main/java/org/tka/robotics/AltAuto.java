package org.tka.robotics;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.tka.robotics.opmode.RedBlueAutonomous;
import org.tka.robotics.opmode.RedBlueOpMode;
import org.tka.robotics.opmode.TeamColor;
import org.tka.robotics.utils.BallScorer;
import org.tka.robotics.utils.hardware.MainBotHardware;

import java.util.Locale;

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

        waitForStart();


        hardware.getUtilities().resetDriveMotors();

        telemetry.log().add("Strafing forward");
        hardware.getUtilities().strafe(2000, 0.8);
        int targetAngle = teamColor == TeamColor.BLUE ? 50 : -50;
        telemetry.log().add(String.format(Locale.ENGLISH, "Turning %d degrees", targetAngle));
        hardware.getUtilities().turnDegrees(0.5, targetAngle);
        telemetry.log().add("Strafing forward");
        hardware.getUtilities().strafe(4250, 0.8);

        telemetry.log().add("Launching ball");
        launchBall();

        sleep(15);

        telemetry.log().add("Dropping in second ball");
        hardware.getBallHolderServo().setPosition(0.5);

        sleep(500);

        telemetry.log().add("Waiting for ball scorer...");
        while (hardware.getBallScorer().getState() != BallScorer.State.WAITING) {
            sleep(10);
        }

        telemetry.log().add("Launching second ball");
        launchBall();
        sleep(500);

        telemetry.log().add("Turning 90 degrees");
        hardware.getUtilities().turnDegrees(0.5, 90);

        telemetry.log().add("Driving forward to cap ball");
        hardware.getUtilities().driveForward(2750, 0.5);


        telemetry.log().add("Done!");
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
            readjustOrientation(INITIAL_HEADING-50);
        while (hardware.getBallScorer().getState() != BallScorer.State.WAITING) {
            idle();
        }

        telemetry.log().add(" + Firing");
        hardware.getBallScorer().launch();
    }


}