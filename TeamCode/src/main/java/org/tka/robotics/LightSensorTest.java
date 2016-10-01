package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by Joshua on 9/24/2016.
 */

@Autonomous(name = "LightSensorTest")
public class LightSensorTest extends LinearOpMode {

    LightSensor lightSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        lightSensor = hardwareMap.lightSensor.get("light_sensor");
        lightSensor.enableLed(true);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Raw", lightSensor.getRawLightDetected());
            telemetry.addData("Normal", lightSensor.getLightDetected());

            telemetry.update();
            idle();
        }



    }
}
