package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.tka.robotics.utils.RobotHardware;
import org.tka.robotics.utils.vuforia.FtcVuforia;

import java.lang.reflect.Field;
import java.util.Arrays;

@Autonomous(name = "VuforiaBeaconScore")
public class VuforiaBeaconScore extends LinearOpMode {

    private RobotHardware hardware;
    private FtcVuforia vuforia;

    private static final float MOTOR_POWER = 0.25F;


    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new RobotHardware(this);
         vuforia = new FtcVuforia(R.id.cameraMonitorViewId, VuforiaLocalizer.CameraDirection.BACK);
        vuforia.setTargets(FtcVuforia.locationMatrix(0, 0, 0, 6 * FtcVuforia.MM_PER_INCH, 0, 0));

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        LightSensor lightSensor = hardwareMap.lightSensor.get("light_sensor");
        lightSensor.enableLed(true);

        telemetry.log().setCapacity(10);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);

        double lowLight = 0;

        for(int i = 0; i < 10; i++){
            lowLight += lightSensor.getLightDetected();
        }

        lowLight /= 10.0D;
        telemetry.log().add("Low Light: "+lowLight);
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
        
        driveSidewaysToTarget(243, 0.3F);
        driveFBToTarget(1292, 0.1f);
        
        telemetry.log().add("Following line");
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

    private void driveSidewaysToTarget(float target, float motorPower) throws InterruptedException {
        telemetry.log().add("Driving to X="+target);
        float[] robotPosition = vuforia.getRobotPosition();
        if(robotPosition == null)
            return;
        while(true){
            if(robotPosition == null)
                break;
            if(robotPosition[0] > target){
                hardware.getFrontLeftMotor().setPower(-motorPower);
                hardware.getBackLeftMotor().setPower(motorPower);
                hardware.getFrontRightMotor().setPower(motorPower);
                hardware.getBackRightMotor().setPower(-motorPower);
            } else {
                hardware.getFrontLeftMotor().setPower(motorPower);
                hardware.getBackLeftMotor().setPower(-motorPower);
                hardware.getFrontRightMotor().setPower(-motorPower);
                hardware.getBackRightMotor().setPower(motorPower);
            }
            if((target - 10 <= robotPosition[0]) && (robotPosition[0]<= target + 10)){
                break;
            }
            robotPosition = vuforia.getRobotPosition();
            idle();
        }
        hardware.stopAllMotors();
    }

    private void driveFBToTarget(float target, float motorPower) throws InterruptedException {
        telemetry.log().add("Driving to Y="+target);
        float[] robotPosition = vuforia.getRobotPosition();
        if(robotPosition == null)
            return;
        while(true){
            if(robotPosition == null)
                break;
            RobotLog.ii("POS", String.valueOf(robotPosition[1]));
            if(robotPosition[1] > target){
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
            if((target - 10 <= robotPosition[1]) && (robotPosition[1]<= target + 10)){
                break;
            }
            robotPosition = vuforia.getRobotPosition();
            idle();
        }
        hardware.stopAllMotors();
    }
}
