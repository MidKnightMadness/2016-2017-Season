package org.tka.robotics.utils.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MainBotHardware extends RobotHardware{

    private DcMotor front_left, front_right, back_left, back_right;

    public MainBotHardware(OpMode opmode) {
        super(opmode);
    }

    @Override
    public void initialize() {
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
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
}
