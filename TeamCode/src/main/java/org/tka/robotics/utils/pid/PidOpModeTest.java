package org.tka.robotics.utils.pid;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "PID Test")
public class PidOpModeTest extends OpMode{

    private PidMotor motor1;
    private PidMotor motor2;
    double power = 0;

    private boolean increasing = true;

    @Override
    public void init() {

    }

    @Override
    public void start() {
        DcMotor m1 = hardwareMap.dcMotor.get("motor");
        DcMotor m2 = hardwareMap.dcMotor.get("motor1");

        motor1 = new PidMotor(m1);
        motor2 = new PidMotor(m2);


        motor1.setVelocity(1);
        motor2.setVelocity(2);
    }

    @Override
    public void stop() {
        PidUpdater.shutdown();
    }

    @Override
    public void loop() {
    }
}
