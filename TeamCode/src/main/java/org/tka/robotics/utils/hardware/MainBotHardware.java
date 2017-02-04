package org.tka.robotics.utils.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.tka.robotics.utils.BallScorer;
import org.tka.robotics.utils.Utilities;

public class MainBotHardware extends RobotHardware{

    private static final double ELEVATOR_RETAINER_INITIAL_POSITION = 0;
    private static final double SEMAPHORE_INITIAL_POSITION = 0;
    private DcMotor front_left, front_right, back_left, back_right, intake, elevator;

    private final Utilities utilities;

    private BallScorer ballScorer;

    private Servo elevatorRetainer, semaphore;

    private UltrasonicSensor ultrasonicSensor;

    private Thread ballScorerThread;

    public MainBotHardware(OpMode opmode) {
        super(opmode);
        this.utilities = new Utilities(parent, this);
    }

    ModernRoboticsI2cGyro gyroSensor;
    LightSensor lightSensor;
    ColorSensor colorSensor;

    @Override
    public void initialize() {
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonic");

        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro_sensor");
        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");


        // TODO: 1/21/2017 AW - Implement ball scorer
        ballScorer = new BallScorer(parent, hardwareMap.dcMotor.get("pinball_motor"),
                hardwareMap.servo.get("pinball_servo"), hardwareMap.touchSensor.get("pinball_touch"));

        //front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        //back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flipped reversals due to hardware gear changes

        for(DcMotor m : hardwareMap.dcMotor){
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);


        semaphore = hardwareMap.servo.get("semaphore");
        elevatorRetainer = hardwareMap.servo.get("elevator_retainer");

        initializeServos();
        telemetry();
    }

    private void initializeServos(){
        elevatorRetainer.setDirection(Servo.Direction.REVERSE);
        semaphore.setPosition(SEMAPHORE_INITIAL_POSITION);
        elevatorRetainer.setPosition(ELEVATOR_RETAINER_INITIAL_POSITION);
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
    public ModernRoboticsI2cGyro getGyroSensor() { return gyroSensor; }

    @Override
    public LightSensor getLightSensor() { return lightSensor; }

    @Override
    public ColorSensor getColorSensor() { return colorSensor; }

    @Override
    public BallScorer getBallScorer() {
        return ballScorer;
    }

    @Override
    public Servo semaphore() {
        return semaphore;
    }

    @Override
    public Servo elevatorRetainer() {
        return elevatorRetainer;
    }

    @Override
    public DcMotor getIntakeMotor() {
        return intake;
    }

    @Override
    public DcMotor getElevatorMotor() {
        return elevator;
    }

    @Override
    public UltrasonicSensor getUltrasonicSensor() {
        return ultrasonicSensor;
    }
}
