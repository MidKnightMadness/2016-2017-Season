package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.Func;

@Autonomous(name = "Ultrasonic Test")
public class UltrasonicTest extends OpMode{

    private UltrasonicSensor ultrasonicSensor;
    private ModernRoboticsI2cColorSensor colorSensor;

    @Override
    public void init() {
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonic");
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Level", new Func<Double>() {
            @Override
            public Double value() {
                return ultrasonicSensor.getUltrasonicLevel();
            }
        });
        telemetry.addData("Color Sensor", new Func<String>() {
            @Override
            public String value() {
                return String.format("%s, %s, %s", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            }
        });
    }

    @Override
    public void loop() {
        telemetry.update();
    }
}
