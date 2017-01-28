package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Joshua on 9/28/2016.
 */

@TeleOp(name = "LittleCarnivalBot")
@Disabled
public class LittleCarnivalBot extends OpMode {
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
        left.setPower(gamepad1.left_stick_y * 0.20);
        right.setPower(gamepad1.right_stick_y * 0.20);

        telemetry.addData("left", gamepad1.left_stick_y);
        telemetry.addData("right", gamepad1.right_stick_y);
        telemetry.update();
    }
}
