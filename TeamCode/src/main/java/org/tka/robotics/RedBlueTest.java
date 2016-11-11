package org.tka.robotics;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.tka.robotics.opmode.RedBlueAutonomous;
import org.tka.robotics.opmode.RedBlueOpMode;

@RedBlueAutonomous(name = "Red Blue OpMode Test")
public class RedBlueTest extends RedBlueOpMode {

    ElapsedTime time;

    @Override
    public void init() {
        time = new ElapsedTime();
    }

    @Override
    public void loop() {
        telemetry.addData("Time elapsed", time.seconds());
        telemetry.addData("Color", teamColor);
        telemetry.update();
    }
}
