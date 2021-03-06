package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.tka.robotics.opmode.RedBlueAutonomous;
import org.tka.robotics.opmode.RedBlueOpMode;
import org.tka.robotics.utils.hardware.SoftwareBotHardware;
import org.tka.robotics.utils.vuforia.FtcVuforia;

import java.lang.reflect.Field;
import java.util.Arrays;

@RedBlueAutonomous(name = "VuforiaBeaconScore")
@Disabled
public class VuforiaBeaconScore extends RedBlueOpMode {

    private static final float MOTOR_POWER = 0.25F;
    private SoftwareBotHardware hardware;
    private FtcVuforia vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        TouchSensor touchSensor;
        touchSensor = hardwareMap.touchSensor.get("touch");
        hardware = new SoftwareBotHardware(this);
        vuforia = new FtcVuforia(R.id.cameraMonitorViewId, VuforiaLocalizer.CameraDirection.FRONT);
        vuforia.setTargets(FtcVuforia.locationMatrix(0, 0, -90, 15 * FtcVuforia.MM_PER_INCH,
                (float) -14.5 * FtcVuforia.MM_PER_INCH, 0));


        telemetry.log().setCapacity(10);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        double lowLight = 0;

        for (int i = 0; i < 10; i++) {
            lowLight += hardware.getLightSensor().getLightDetected();
        }

        lowLight /= 10.0D;
        telemetry.log().add("Low Light: " + lowLight);
        telemetry.log().add("Initialized and ready!");

        waitForStart();

        vuforia.setTrackingEnabled(true);

        telemetry.log().add("Starting driving until we find a location from the target");
        // Drive sideways until we get a position from the targets
        while (vuforia.getRobotPosition() == null) {
            logPositionData(vuforia);
            driveBackLeftDiagonal(0.30f);
            idle();
        }
        telemetry.log().add("Found target, stopping");
        hardware.stopAllMotors();

        sleep(500);

        ///////////////////////////////
        //NATHAN B CHANGED 680 to 570//
        ///////////////////////////////
        driveToTarget(550, 1350, 0.5F); //580, 1318
        hardware.stopAllMotors();

        
/*        telemetry.log().add("Following line");
        while((colorSensor.red() <= 1) && colorSensor.blue() <= 1) {
            if(lightSensor.getLightDetected() < 0.35) {
                hardware.getFrontLeftMotor().setPower(0);
                hardware.getFrontRightMotor().setPower(0.15);
                hardware.getBackLeftMotor().setPower(0);
                hardware.getBackRightMotor().setPower(0.15);
            } else {
                hardware.getFrontLeftMotor().setPower(0.15);
                hardware.getFrontRightMotor().setPower(0);
                hardware.getBackLeftMotor().setPower(0.15);
                hardware.getBackRightMotor().setPower(0);
            }
            telemetry.addData("Light", lightSensor);
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
        hardware.stopAllMotors();*/

        /*
        Trials:


         */

        /*sleep(500);

        hardware.getLightSensor().enableLed(true);
        hardware.getUtilities().sideLineFollow();
        hardware.stopAllMotors();

        ////////////////////////
        //ADDED BEACON SCORING//
        ////////////////////////


        if(teamColor == TeamColor.BLUE)
            hardware.getUtilities().detectBeaconColorAndAdjustBlue();
        if(teamColor == TeamColor.RED)
            hardware.getUtilities().detectBeaconColorAndAdjustRed();

        while (!touchSensor.isPressed()) {
            this.hardware.getFrontLeftMotor().setPower(0.4);
            this.hardware.getFrontRightMotor().setPower(-0.4);
            this.hardware.getBackLeftMotor().setPower(-0.4);
            this.hardware.getBackRightMotor().setPower(0.4);
            idle();
        }*/

        while (opModeIsActive())
            idle();
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

    private void driveToTarget(float targetX, float targetY, float motorPower) throws InterruptedException {
        float[] robotPosition = vuforia.getRobotPosition();

        float frontLeftMotorPower = 0;
        float frontRightMotorPower = 0;
        float backLeftMotorPower = 0;
        float backRightMotorPower = 0;
        float deltaX;
        float deltaY;
        float multiplierX;
        float multiplierY;
        if (robotPosition == null)
            return;
        while ((Math.abs(robotPosition[0] - targetX) > 15) || (Math.abs(robotPosition[1] - targetY) > 15)) {
            robotPosition = vuforia.getRobotPosition();
            logPositionData(vuforia);

            deltaX = Math.abs(targetX - robotPosition[0]);
            multiplierX = Range.clip(deltaX / 750, 0.10F, 0.3F);
            if (robotPosition[0] < targetX) {
                frontLeftMotorPower = motorPower * multiplierX;
                frontRightMotorPower = motorPower * multiplierX;
                backLeftMotorPower = motorPower * multiplierX;
                backRightMotorPower = motorPower * multiplierX;
            } else {
                frontLeftMotorPower = -motorPower * multiplierX;
                frontRightMotorPower = -motorPower * multiplierX;
                backLeftMotorPower = -motorPower * multiplierX;
                backRightMotorPower = -motorPower * multiplierX;
            }

            deltaY = Math.abs(targetY - robotPosition[1]);
            multiplierY = Range.clip(deltaY / 750, 0.10F, 0.3F);
            if (robotPosition[1] < targetY) {
                frontLeftMotorPower += -motorPower * multiplierY;
                frontRightMotorPower += motorPower * multiplierY;
                backLeftMotorPower += motorPower * multiplierY;
                backRightMotorPower += -motorPower * multiplierY;
            } else {
                frontLeftMotorPower += motorPower * multiplierY;
                frontRightMotorPower += -motorPower * multiplierY;
                backLeftMotorPower += -motorPower * multiplierY;
                backRightMotorPower += motorPower * multiplierY;
            }



            hardware.getFrontLeftMotor().setPower(frontLeftMotorPower);
            hardware.getFrontRightMotor().setPower(frontRightMotorPower);
            hardware.getBackLeftMotor().setPower(backLeftMotorPower);
            hardware.getBackRightMotor().setPower(backRightMotorPower);

//            telemetry.addData("x", robotPosition[0]);
//            telemetry.addData("y", robotPosition[1]);

            idle();
        }
    }

    private void driveForward(float motorPower) {
        hardware.getFrontLeftMotor().setPower(motorPower);
        hardware.getFrontRightMotor().setPower(motorPower);
        hardware.getBackLeftMotor().setPower(motorPower);
        hardware.getBackRightMotor().setPower(motorPower);
    }

    private void driveBackLeftDiagonal(float motorPower) {
        hardware.getFrontLeftMotor().setPower(-motorPower);
        hardware.getFrontRightMotor().setPower(0);
        hardware.getBackLeftMotor().setPower(0);
        hardware.getBackRightMotor().setPower(-motorPower);
    }

    private float scalePower(float power, float target, float actual){
        float delta = target - actual;
        float newPower = (float) (power * (delta * 0.5));
        telemetry.addData("scaledPower",newPower);
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
}
