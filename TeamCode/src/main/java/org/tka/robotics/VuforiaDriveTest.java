package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.tka.robotics.utils.RobotHardware;
import org.tka.robotics.utils.vuforia.FtcVuforia;

import java.lang.reflect.Field;
import java.util.Arrays;

@Autonomous(name = "VuforiaDriveTest")
public class VuforiaDriveTest extends LinearOpMode {

    private RobotHardware hardware;

    private static float MOTOR_POWER = 0.5f;


    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(this);
        FtcVuforia vuforia = new FtcVuforia(R.id.cameraMonitorViewId, VuforiaLocalizer.CameraDirection.BACK);
        vuforia.setTargets(FtcVuforia.locationMatrix(0, 0, 0, 6 * FtcVuforia.MM_PER_INCH, 0, 0));

        telemetry.log().setCapacity(10);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        telemetry.log().add("Initialized and ready!");
        waitForStart();

        vuforia.setTrackingEnabled(true);

        telemetry.log().add("Staring driving until we find a location from the target");
        // Drive sideways until we get a position from the targets
        while (vuforia.getRobotPosition() == null) {
            logPositionData(vuforia);
            setAllMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveSideways(-MOTOR_POWER);
            idle();
        }
        telemetry.log().add("Found target, stopping");
        hardware.stopAllMotors();
        // Drive to 175, 1534

        // Drive X
        telemetry.log().add("Driving to X=175");
        float[] robotPosition = vuforia.getRobotPosition();
        while (!inRange(175, 5, robotPosition[0])) {
            logPositionData(vuforia);
            if(vuforia.getRobotPosition() == null) {
                break;
            }
            robotPosition = vuforia.getRobotPosition();
            if (robotPosition[0] < 175) {
                driveSideways(-MOTOR_POWER);
            } else if (robotPosition[0] > 175) {
                driveSideways(MOTOR_POWER);
            }
            idle();
        }
        hardware.stopAllMotors();

        telemetry.log().add("Driving to Y=1534");
        // Drive Y
        while(!inRange(1534, 5, robotPosition[1])){
            logPositionData(vuforia);
            if(vuforia.getRobotPosition() == null)
                break;
            robotPosition = vuforia.getRobotPosition();
            if (robotPosition[0] < 1534) {
                driveStraight(MOTOR_POWER);
            } else if (robotPosition[0] > 1534) {
                driveStraight(-MOTOR_POWER);
            }
            idle();
        }
        hardware.stopAllMotors();

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

    private void driveStraight(double power){
        hardware.getFrontLeftMotor().setPower(power);
        hardware.getBackLeftMotor().setPower(power);
        hardware.getFrontRightMotor().setPower(power);
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

    private boolean inRange(float target, float tolerance, float actual) {
        // target - tol <= actual <= target + tol
        return (target - tolerance <= actual) && (actual <= target + tolerance);
    }
}
