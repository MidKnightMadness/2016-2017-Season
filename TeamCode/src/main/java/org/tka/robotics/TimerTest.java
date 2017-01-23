package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Joshua on 10/15/2016.
 */

@TeleOp(name = "TimerTest")
public class TimerTest extends OpMode {

    @Override
    public void init() {
        this.resetStartTime();
    }

    @Override
    public void loop() {
        telemetry.addData("timer", this.getRuntime());
    }
}