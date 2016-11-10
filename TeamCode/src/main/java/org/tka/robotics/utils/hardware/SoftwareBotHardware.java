package org.tka.robotics.utils.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.tka.robotics.utils.Utilities;

import java.util.Map;

/**
 * Hardware abstraction for interaction with Carnival Bot
 */
public class SoftwareBotHardware extends RobotHardware{

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private final Utilities utilities;

    public SoftwareBotHardware(OpMode opMode) {
        super(opMode);
        this.utilities = new Utilities(parent, this);
    }

    ModernRoboticsI2cGyro gyro;
    LightSensor lightSensor;
    ColorSensor colorSensor;

    /**
     * Initializes all the hardware and telemetry on the robot
     */
    @Override
    public void initialize() {
        telemetry();
        // Reset all the motor configurations
        for (Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            m.getValue().resetDeviceConfigurationForOpMode();
            m.getValue().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        frontLeft = hardwareMap.dcMotor.get("left_front");
        frontRight = hardwareMap.dcMotor.get("right_front");
        backLeft = hardwareMap.dcMotor.get("left_back");
        backRight = hardwareMap.dcMotor.get("right_back");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Gets the front left motor
     *
     * @return A {@link DcMotor} corresponding to the front left motor
     */
    @Override
    public DcMotor getFrontLeftMotor() {
        return this.frontLeft;
    }

    /**
     * Gets the front right motor
     *
     * @return A {@link DcMotor} corresponding to the front right motor
     */
    @Override
    public DcMotor getFrontRightMotor() {
        return this.frontRight;
    }

    /**
     * Gets the back left motor
     *
     * @return A {@link DcMotor} corresponding to the back left motor
     */
    @Override
    public DcMotor getBackLeftMotor() {
        return this.backLeft;
    }

    /**
     * Gets the back right motor
     *
     * @return A {@link DcMotor} corresponding to the back right motor
     */
    @Override
    public DcMotor getBackRightMotor() {
        return this.backRight;
    }

    @Override
    public Utilities getUtilities() {
        return utilities;
    }

    @Override
    public GyroSensor getGyroSensor() { return gyro; }

    @Override
    public LightSensor getLightSensor() { return lightSensor; }

    @Override
    public ColorSensor getColorSensor() { return colorSensor; }

    /**
     * Stops <b>all</b> motors on the robot
     */
    @Override
    public void stopAllMotors() {
        for (Map.Entry<String, DcMotor> m : this.hardwareMap.dcMotor.entrySet()) {
            m.getValue().setPower(0);
        }
    }


}
