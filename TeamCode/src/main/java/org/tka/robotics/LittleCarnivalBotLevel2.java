package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Random;

/**
 * Created by Joshua on 9/28/2016.
 */

@TeleOp(name = "LittleCarnivalBotFunVersion")
@Disabled
public class LittleCarnivalBotLevel2 extends OpMode {
    private DcMotor left, right;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        left.resetDeviceConfigurationForOpMode();
        right.resetDeviceConfigurationForOpMode();

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        left.setPower(gamepad1.left_stick_y * myRandom(0.1, 1.0));
        right.setPower(gamepad1.right_stick_y * myRandom(0.1, 1.0));

        telemetry.addData("left", gamepad1.left_stick_y);
        telemetry.addData("right", gamepad1.right_stick_y);
        telemetry.update();
    }

    double myRandom(double min, double max) {
        Random r = new Random();
        return (r.nextInt((int)((max-min)*10+1))+min*10) / 10.0;
    }
}
