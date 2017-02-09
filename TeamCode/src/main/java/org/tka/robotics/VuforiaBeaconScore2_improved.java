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

@RedBlueAutonomous(name = "Vuforia Beacon Score improved")
public class VuforiaBeaconScore2_improved extends RedBlueOpMode {

    private static final float MOTOR_POWER = 0.25F;
    private MainBotHardware hardware;
    private FtcVuforia vuforia;
    private static float INITIAL_HEADING;
    int heading = 0;

    private int red, green, blue;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new MainBotHardware(this);
        vuforia = new FtcVuforia(R.id.cameraMonitorViewId, VuforiaLocalizer.CameraDirection.FRONT);
        vuforia.setTargets(FtcVuforia.locationMatrix(0, 0, -90, 15 * FtcVuforia.MM_PER_INCH,
                (float) -14.5 * FtcVuforia.MM_PER_INCH, 0));


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

        red = hardware.getColorSensor().red();
        blue = hardware.getColorSensor().blue();
        green = hardware.getColorSensor().green();

        hardware.getBallScorer().start();

        waitForStart();


        vuforia.setTrackingEnabled(true);

        hardware.getUtilities().resetDriveMotors();


        telemetry.log().add("Driving into place");
        hardware.getUtilities().strafe(3000, 1);
        telemetry.log().add("Launching Ball");
        launchBall();
        sleep(15);

        telemetry.log().add("Driving to beacon");
        hardware.getUtilities().driveForward(-4500, 1);

        hardware.getUtilities().turnDegrees(0.50, -90);

        // Drive until we have a lock on the target
        while (vuforia.getRobotPosition() == null) {
            hardware.getUtilities().setAllMotors(-0.25);
            idle();
        }

        hardware.stopAllMotors();

        readjustOrientation(INITIAL_HEADING - 90);

        telemetry.log().add("Driving to white line");
        hardware.getLightSensor().enableLed(true);
        while (hardware.getLightSensor().getLightDetected() < 0.4) {
            hardware.getUtilities().setAllMotors(-0.25);
            idle();
        }

        hardware.stopAllMotors();

        telemetry.log().add("Pushing Beacon");
        pushBeacon();

        telemetry.log().add("Driving to second beacon");
        driveToSecondBeacon();

        telemetry.log().add("Pushing beacon");
        pushBeacon();

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

    private void logPositionData(FtcVuforia vuforia) {
        if (vuforia.getRobotPosition() != null)
            telemetry.addData("Pos", Arrays.toString(vuforia.getRobotPosition()));
        else
            telemetry.addData("Pos", "Unknown");
        telemetry.update();
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

    private void driveToTargetBlue(float targetY/*, float motorPower*/) throws InterruptedException {
        float[] robotPosition = vuforia.getRobotPosition();

        float frontLeftMotorPower = 0;
        float frontRightMotorPower = 0;
        float backLeftMotorPower = 0;
        float backRightMotorPower = 0;
        //float deltaX;
        float deltaY;
        //float multiplierX;
        float multiplierY;
        if (robotPosition == null)
            return;

        hardware.getLightSensor().enableLed(true);

        while (hardware.getLightSensor().getLightDetected() < 0.4) {
            robotPosition = vuforia.getRobotPosition();
            logPositionData(vuforia);


            frontLeftMotorPower = -0.08F;
            frontRightMotorPower = -0.08F;
            backLeftMotorPower = -0.08F;
            backRightMotorPower = -0.08F;


            deltaY = Math.abs(targetY - robotPosition[1]);
            multiplierY = Range.clip(deltaY / 750, 0.1F, 0.2F);
            if (Math.abs(targetY - robotPosition[1]) > 25) {
                if (robotPosition[1] < targetY) {
                    frontLeftMotorPower -= multiplierY;
                    frontRightMotorPower += multiplierY;
                    backLeftMotorPower += multiplierY;
                    backRightMotorPower -= multiplierY;
                } else {
                    frontLeftMotorPower += multiplierY;
                    frontRightMotorPower -= multiplierY;
                    backLeftMotorPower -= multiplierY;
                    backRightMotorPower += multiplierY;
                }
            }


            hardware.getFrontLeftMotor().setPower(frontLeftMotorPower);
            hardware.getFrontRightMotor().setPower(frontRightMotorPower);
            hardware.getBackLeftMotor().setPower(backLeftMotorPower);
            hardware.getBackRightMotor().setPower(backRightMotorPower);

            telemetry.addData("front left", frontLeftMotorPower);
            telemetry.addData("front right", frontRightMotorPower);
            telemetry.addData("back left", backLeftMotorPower);
            telemetry.addData("back right", backRightMotorPower);

            telemetry.addData("light sensor", hardware.getLightSensor().getLightDetected());

            idle();
        }
    }

    private void driveToTargetRed(float targetX/*, float motorPower*/) throws InterruptedException {
        float[] robotPosition = vuforia.getRobotPosition();

        float frontLeftMotorPower = 0;
        float frontRightMotorPower = 0;
        float backLeftMotorPower = 0;
        float backRightMotorPower = 0;
        float deltaX;
        //float deltaY;
        float multiplierX;
        //float multiplierY;
        if (robotPosition == null)
            return;

        hardware.getLightSensor().enableLed(true);

        while (hardware.getLightSensor().getLightDetected() < 0.4) {
            robotPosition = vuforia.getRobotPosition();
            logPositionData(vuforia);


            frontLeftMotorPower = 0.06F; // was 0.08
            frontRightMotorPower = 0.06F; // was 0.08
            backLeftMotorPower = 0.06F; // was 0.08
            backRightMotorPower = 0.06F; // was 0.08


            deltaX = Math.abs(targetX - robotPosition[0]);
            multiplierX = Range.clip(deltaX / 750, 0.1F, 0.3F);
            if (Math.abs(targetX - robotPosition[0]) > 25) {
                if (robotPosition[0] < targetX) {
                    frontLeftMotorPower += multiplierX;
                    frontRightMotorPower -= multiplierX;
                    backLeftMotorPower -= multiplierX;
                    backRightMotorPower += multiplierX;
                } else {
                    frontLeftMotorPower -= multiplierX;
                    frontRightMotorPower += multiplierX;
                    backLeftMotorPower += multiplierX;
                    backRightMotorPower -= multiplierX;
                }
            }


            hardware.getFrontLeftMotor().setPower(frontLeftMotorPower);
            hardware.getFrontRightMotor().setPower(frontRightMotorPower);
            hardware.getBackLeftMotor().setPower(backLeftMotorPower);
            hardware.getBackRightMotor().setPower(backRightMotorPower);

            telemetry.addData("front left", frontLeftMotorPower);
            telemetry.addData("front right", frontRightMotorPower);
            telemetry.addData("back left", backLeftMotorPower);
            telemetry.addData("back right", backRightMotorPower);

            telemetry.addData("light sensor", hardware.getLightSensor().getLightDetected());

            idle();
        }
        readjustOrientation(INITIAL_HEADING + 90);
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

    private float scalePower(float power, float target, float actual) {
        float delta = target - actual;
        float newPower = (float) (power * (delta * 0.5));
        telemetry.addData("scaledPower", newPower);
        telemetry.addData("DeltaPosition", newPower);
        telemetry.update();
        return Range.clip(newPower, -power, power);
    }

    private void driveFBToTarget(float target, float motorPower) throws InterruptedException {
        telemetry.log().add("Driving to Y=" + target);
        float[] robotPosition = vuforia.getRobotPosition();
        if (robotPosition == null)
            return;
        while (true) {
            if (robotPosition == null)
                break;
            RobotLog.ii("POS", String.valueOf(robotPosition[1]));
            if (robotPosition[1] > target) {
                hardware.getFrontLeftMotor().setPower(-motorPower);
                hardware.getFrontRightMotor().setPower(-motorPower);
                hardware.getBackRightMotor().setPower(-motorPower);
                hardware.getBackLeftMotor().setPower(-motorPower);
            } else {
                hardware.getFrontLeftMotor().setPower(motorPower);
                hardware.getFrontRightMotor().setPower(motorPower);
                hardware.getBackRightMotor().setPower(motorPower);
                hardware.getBackLeftMotor().setPower(motorPower);
            }
            if ((target - 10 <= robotPosition[1]) && (robotPosition[1] <= target + 10)) {
                break;
            }
            robotPosition = vuforia.getRobotPosition();
            idle();
        }
        hardware.stopAllMotors();
    }

    private void pushBeacon() throws InterruptedException {
        telemetry.log().add(" + Following Line");
        hardware.getUtilities().sideLineFollow();


        readjustOrientation(INITIAL_HEADING - 90);

//        sleep(500);

        telemetry.log().add(" + Adjusting color to " + teamColor);
        if (teamColor == TeamColor.BLUE)
            hardware.getUtilities().detectBeaconColorAndAdjustBlue();
        if (teamColor == TeamColor.RED)
            hardware.getUtilities().detectBeaconColorAndAdjustRed();

        //boolean eitherTouchPressed = touchSensor1.isPressed() || touchSensor2.isPressed();


        ElapsedTime time = new ElapsedTime();

        // run until the wall until either one of touch sensors is pressed or the timer reaches 5 seconds

        telemetry.log().add(" + Pushing beacon");
        while (time.seconds() < 2.0) {
            this.hardware.getFrontLeftMotor().setPower(-0.2);
            this.hardware.getFrontRightMotor().setPower(0.2);
            this.hardware.getBackLeftMotor().setPower(0.2);
            this.hardware.getBackRightMotor().setPower(-0.2);
            telemetry.addData("timer", this.getRuntime());
            idle();
        }

        hardware.stopAllMotors();

        telemetry.log().add(" + Driving back");
        hardware.getUtilities().strafe(1500, 0.3);
        hardware.stopAllMotors();
    }

    private void driveToSecondBeacon() throws InterruptedException {
        if (teamColor == TeamColor.BLUE)
            hardware.getUtilities().driveForward(-2750, 0.75);

        if (teamColor == TeamColor.RED)
            hardware.getUtilities().driveForward(2750, 0.75);

        while (hardware.getLightSensor().getLightDetected() < 0.4) {
            hardware.getUtilities().setAllMotors(teamColor == TeamColor.BLUE ? -0.25 : 0.25);
            idle();
        }
    }
}