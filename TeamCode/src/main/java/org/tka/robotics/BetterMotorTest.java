package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Func;

import java.util.Map;

@TeleOp(name = "Better Motor Test")
public class BetterMotorTest extends OpMode {

    private String[] motorNames;
    private DcMotor[] motors;
    private int selectedMotor = 0;

    private boolean nextMotorPressed = false;

    @Override
    public void init() {
        motorNames = new String[hardwareMap.dcMotor.size()];
        motors = new DcMotor[hardwareMap.dcMotor.size()];
        int index = 0;
        for (Map.Entry<String, DcMotor> m : hardwareMap.dcMotor.entrySet()) {
            motorNames[index] = m.getKey();
            motors[index] = m.getValue();
            index++;
        }
        telemetry.addData("Selected Motor", new Func<String>() {
            @Override
            public String value() {
                try{
                    return motorNames[selectedMotor];
                } catch (IndexOutOfBoundsException e){
                    return "Unknown";
                }
            }
        });
    }

    @Override
    public void loop() {
        if (!nextMotorPressed && gamepad1.x) {
            nextMotorPressed = true;
            motors[selectedMotor].setPower(0);
            selectedMotor++;
            if (selectedMotor >= motorNames.length) {
                selectedMotor = 0;
            }
        }
        if(nextMotorPressed && !gamepad1.x){
            nextMotorPressed = false;
        }
        motors[selectedMotor].setPower(gamepad1.left_stick_y);
    }
}
