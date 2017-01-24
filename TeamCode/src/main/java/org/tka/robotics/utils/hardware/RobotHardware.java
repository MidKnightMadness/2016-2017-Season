package org.tka.robotics.utils.hardware;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.tka.robotics.utils.BallScorer;
import org.tka.robotics.utils.Utilities;

import java.util.Map;

public abstract class RobotHardware {

    protected OpMode parent;
    protected HardwareMap hardwareMap;

    public RobotHardware(OpMode opmode){
        this.parent = opmode;
        this.hardwareMap = opmode.hardwareMap;
        preInit();
    }

    private void preInit(){
        Log.i("RobotHardware", "Running PreInit");
        for(DcMotor m : hardwareMap.dcMotor){
            m.resetDeviceConfigurationForOpMode();
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        Log.i("RobotHardware", "PreInit complete!");
        Log.i("RobotHardware", "Running Initialization");
        initialize();
        Log.i("RobotHardware", "Initialization complete!");
    }

    public abstract void initialize();

    public abstract DcMotor getFrontLeftMotor();

    public abstract DcMotor getBackLeftMotor();

    public abstract DcMotor getFrontRightMotor();

    public abstract DcMotor getBackRightMotor();

    public abstract Utilities getUtilities();

    public abstract GyroSensor getGyroSensor();

    public abstract LightSensor getLightSensor();

    public abstract ColorSensor getColorSensor();

    public abstract BallScorer getBallScorer();

    public void stopAllMotors(){
        for(DcMotor m :hardwareMap.dcMotor){
            m.setPower(0);
        }
    }
    /**
     * Initializes telemetry to be sent to the driver station.
     * <br><br>
     * <b>Displays the following data:</b>
     * <ul>
     * <li>All motor's power</li>
     * <li>All motor's current encoder position</li>
     * <li>All motor's target encoder position</li>
     * </ul>
     */
    public void telemetry() {
        for (final Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            this.parent.telemetry.addData(m.getKey() + " Power", new Func<Double>() {
                @Override
                public Double value() {
                    return m.getValue().getPower();
                }
            });
            this.parent.telemetry.addData(m.getKey() + " Position", new Func<Integer>() {
                @Override
                public Integer value() {
                    return m.getValue().getCurrentPosition();
                }
            });
            this.parent.telemetry.addData(m.getKey() + " Target", new Func<Integer>() {
                @Override
                public Integer value() {
                    return m.getValue().getTargetPosition();
                }
            });
        }
    }
}
