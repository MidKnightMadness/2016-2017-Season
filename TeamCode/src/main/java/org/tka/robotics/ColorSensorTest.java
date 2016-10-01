package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        waitForStart();

        colorSensor.enableLed(true);

        while(opModeIsActive()) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Green", colorSensor.green());

            if(colorSensor.blue() > colorSensor.red()) {
                telemetry.addData("Status", "BLUE!!");
            }
            if (colorSensor.red() > colorSensor.blue()) {
                telemetry.addData("Status", "RED!!");
            }

            telemetry.update();
            idle();
        }

    }

}