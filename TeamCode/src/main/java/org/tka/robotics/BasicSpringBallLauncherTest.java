package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Joshua on 10/15/2016.
 */


@Autonomous(name = "BallSpringLauncherTest")
public class BasicSpringBallLauncherTest extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {



        motor.setPower(Range.clip(gamepad1.left_stick_y, -0.5, 0.5));
        //waitForStart();



        /*
        while(motor.getCurrentPosition() < 10000) {
            motor.setPower(0.4);
            Thread.yield();
        }
        motor.setPower(0);
        Thread.sleep(3000);
        */

        /*motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motor.getCurrentPosition() > -5000) {
            motor.setPower(-0.4);
            Thread.yield();
        }
        motor.setPower(0);*/
    }
}
