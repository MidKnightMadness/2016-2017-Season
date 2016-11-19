package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.tka.robotics.utils.hardware.MainBotHardware;

@TeleOp(name = "Main Bot Teleop")
public class MainBotTeleop extends OpMode {

    /**
     * The intake motor for collecting particles off the field
     */
    private DcMotor intake;

    private DcMotor elevator;

    /**
     * The main robot hardware
     */
    private MainBotHardware hardware;

    private boolean intakeToggle = false;
    private boolean intakeTogglePressed = true;



    @Override
    public void init() {
        hardware = new MainBotHardware(this);
        intake = hardwareMap.dcMotor.get("intake");
        elevator = hardwareMap.dcMotor.get("elevator");
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.resetDeviceConfigurationForOpMode();
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        tankDrive(scaleInput(-gamepad1.left_stick_y), scaleInput(-gamepad1.right_stick_y));
        omniDrive();
        updateIntake();
        intake.setPower(intakeToggle ? 1.0F : 0.0F);




        updateElevator();
    }

    private static final int ELEVATOR_UP_POSITION = 18000; // was 16000

    private void updateElevator() {
        if (gamepad1.b) {
            elevator.setPower(0.75F);
            elevator.setTargetPosition(0);
        } if (gamepad1.y) {
            elevator.setPower(0.75F);
            elevator.setTargetPosition(ELEVATOR_UP_POSITION);
        } else {
            elevator.setPower(0);
        }
    }

    // TODO: Broken, fix soon™
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

    /**
     * Tank Drive (Run the left and right motors in the same direction
     *
     * @param leftPower  The power to the left motors
     * @param rightPower The power to the right motors
     */
    private void tankDrive(float leftPower, float rightPower) {
        if (!(gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right)) {
            hardware.getFrontLeftMotor().setPower(leftPower);
            hardware.getBackLeftMotor().setPower(leftPower);

            hardware.getFrontRightMotor().setPower(rightPower);
            hardware.getBackRightMotor().setPower(rightPower);
        }
    }

    private void omniDrive() {
        if ( gamepad1.dpad_up && gamepad1.dpad_right ) {
            setEachMotor(1, 0, 0, 1);
        } else if ( gamepad1.dpad_right && gamepad1.dpad_down ) {
            setEachMotor(0, -1, -1, 0);
        } else if (gamepad1.dpad_down && gamepad1.dpad_left ) {
            setEachMotor(-1, 0, 0, -1);
        } else if (gamepad1.dpad_left && gamepad1.dpad_up ) {
            setEachMotor(0, 1, 1, 0);
        } else if (gamepad1.dpad_up) {
            setEachMotor(1, 1, 1, 1);
        } else if (gamepad1.dpad_right) {
            setEachMotor(1, -1, -1, 1);
        } else if (gamepad1.dpad_down) {
            setEachMotor(-1, -1, -1, -1);
        } else if (gamepad1.dpad_left) {
            setEachMotor(-1, 1, 1, -1);
        }
    }

    private void updateIntake() {
        if (gamepad1.a) {
            if (!intakeTogglePressed) {
                intakeTogglePressed = true;
                intakeToggle = !intakeToggle;
            }
        } else {
            intakeTogglePressed = false;
        }
    }

    private void setEachMotor(float frontLeft, float frontRight, float backLeft, float backRight) {
        hardware.getFrontLeftMotor().setPower(frontLeft);
        hardware.getFrontRightMotor().setPower(frontRight);

        hardware.getBackLeftMotor().setPower(backLeft);
        hardware.getBackRightMotor().setPower(backRight);
    }
}
