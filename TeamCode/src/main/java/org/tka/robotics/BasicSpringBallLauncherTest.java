package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Joshua on 10/15/2016.
 */


@Autonomous(name = "BallSpringLauncherTest")
public class BasicSpringBallLauncherTest extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motor.getCurrentPosition() < 50) {
            motor.setPower(0.5);
            Thread.yield();
        }
        motor.setPower(0);
        Thread.sleep(3000);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(motor.getCurrentPosition() > -50) {
            motor.setPower(-0.5);
            Thread.yield();
        }
        motor.setPower(0);
    }
}
