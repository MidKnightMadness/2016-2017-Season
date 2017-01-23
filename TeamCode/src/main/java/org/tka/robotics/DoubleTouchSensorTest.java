package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.tka.robotics.utils.hardware.MainBotHardware;

/**
 * Created by Joshua on 10/15/2016.
 */

@TeleOp(name = "DoubleTouchSensorTest")
public class DoubleTouchSensorTest extends OpMode {

    private MainBotHardware hardware;
    TouchSensor touchSensor1;
    TouchSensor touchSensor2;

    @Override
    public void init() {

        touchSensor1 = hardwareMap.touchSensor.get("touch1");
        touchSensor2 = hardwareMap.touchSensor.get("touch2");

    }

    @Override
    public void loop() {

        telemetry.addData("touch1", touchSensor1.isPressed());
        telemetry.addData("touch2", touchSensor2.isPressed());

    }
}