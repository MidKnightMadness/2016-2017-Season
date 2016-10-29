package org.tka.robotics.utils.pid;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "PID Test")
public class PidOpModeTest extends OpMode{



    private PidMotor motor;
    private PidMotor motor2;

    @Override
    public void init() {
        motor = new PidMotor(hardwareMap, "motor1");
        motor2 = new PidMotor(hardwareMap, "motor");
    }

    @Override
    public void start() {
        motor.addTelemetry(this);
        motor2.addTelemetry(this);
    }

    @Override
    public void stop() {
        PidUpdater.shutdown();
    }

    @Override
    public void loop() {
        motor2.setPower(gamepad1.right_stick_y);
        motor.setPower(gamepad1.left_stick_y);
        if(gamepad1.a){
            motor2.setPosition(0);
            motor.setPosition(0);
        }
    }
}
