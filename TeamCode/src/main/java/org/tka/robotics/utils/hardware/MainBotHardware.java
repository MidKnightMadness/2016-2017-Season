package org.tka.robotics.utils.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.Utilities;

public class MainBotHardware extends RobotHardware{

    private DcMotor front_left, front_right, back_left, back_right;

    private final Utilities utilities;

    public MainBotHardware(OpMode opmode) {
        super(opmode);
        this.utilities = new Utilities(parent, this);
    }

    ModernRoboticsI2cGyro gyro;
    LightSensor lightSensor;
    ColorSensor colorSensor;

    @Override
    public void initialize() {
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry();
    }

    @Override
    public DcMotor getFrontLeftMotor() {
        return front_left;
    }

    @Override
    public DcMotor getBackLeftMotor() {
        return back_left;
    }

    @Override
    public DcMotor getFrontRightMotor() {
        return front_right;
    }

    @Override
    public DcMotor getBackRightMotor() {
        return back_right;
    }

    @Override
    public Utilities getUtilities() {
        return utilities;
    }

    @Override
    public GyroSensor getGyroSensor() { return null; }

    @Override
    public LightSensor getLightSensor() { return lightSensor; }

    @Override
    public ColorSensor getColorSensor() { return colorSensor; }
}
