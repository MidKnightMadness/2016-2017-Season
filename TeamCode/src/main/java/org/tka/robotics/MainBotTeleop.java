package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.tka.robotics.utils.hardware.MainBotHardware;

@TeleOp(name = "Main Bot Teleop")
public class MainBotTeleop extends OpMode {

    private DcMotor intake;

    private MainBotHardware hardware;

    private boolean intakeToggle = false;
    private boolean intakeTogglePressed = true;


    @Override
    public void init() {
        hardware = new MainBotHardware(this);
        intake = hardwareMap.dcMotor.get("intake");

        intake.resetDeviceConfigurationForOpMode();
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double[] motorPwr = calculateMotorValues(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y);
        hardware.getFrontLeftMotor().setPower(motorPwr[0]);
        hardware.getFrontRightMotor().setPower(motorPwr[1]);
        hardware.getBackLeftMotor().setPower(motorPwr[2]);
        hardware.getBackRightMotor().setPower(motorPwr[3]);

        updateIntake();
        intake.setPower(intakeToggle? 1.0F : 0.0F);
    }

    private double[] calculateMotorValues(float joyLX, float joyLY, float joyRX, float joyRY) {
        double[] motorPower = new double[]{0, 0, 0, 0};
        joyRY = scaleInput(joyRY);
        joyLY = scaleInput(joyLY);
        joyRX = scaleInput(joyRX);
        joyLX = scaleInput(joyLX);

        float modX = -(joyLX + joyRX) / 2;
        telemetry.addData("modX: ", modX);
        // front_left
        motorPower[0] = modX + joyLY;
        // front_right
        motorPower[1] = modX - joyRY;
        // back_left
        motorPower[2] = modX - joyLY;
        // back_right
        motorPower[3] = modX + joyRY;

        // limit range
        for (int i = 0; i < motorPower.length; i++) {
            motorPower[i] = Range.clip(motorPower[i], -1F, 1F);
        }
        return motorPower;
    }

    private float scaleInput(float input) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43,
                0.50, 0.60, 0.72, 0.85, 1.0, 1.0};
        int index = (int) (input * 16);
        if (index < 0)
            index = -index;
        else if (index > 16)
            index = 16;
        float scaled;
        if (input < 0)
            scaled = (float) -scaleArray[index];
        else
            scaled = (float) scaleArray[index];
        return scaled / 2;
    }

    private void updateIntake(){
        if(gamepad1.a){
            if(!intakeTogglePressed) {
                intakeTogglePressed = true;
                intakeToggle = !intakeToggle;
            }
        } else {
            intakeTogglePressed = false;
        }
    }
}
