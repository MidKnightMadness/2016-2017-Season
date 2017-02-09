package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.tka.robotics.utils.BallScorer;
import org.tka.robotics.utils.hardware.MainBotHardware;

@TeleOp(name = "Main Bot Teleop")
public class MainBotTeleop extends OpMode {

    private static final int FRONT_LEFT = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT = 2;
    private static final int BACK_RIGHT = 3;

    private static final double ELEVATOR_CLOSE = 0;
    private static final double ELEVATOR_OPEN = 1;

    /**
     * The main robot hardware
     */
    private MainBotHardware hardware;

    private boolean intakeToggle = false;
    private boolean intakeTogglePressed = true;


    @Override
    public void init() {
        hardware = new MainBotHardware(this);


        hardware.getElevatorMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.getIntakeMotor().resetDeviceConfigurationForOpMode();
        hardware.getIntakeMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.getIntakeMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        hardware.getElevatorMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        hardware.elevatorRetainer().setDirection(Servo.Direction.REVERSE);
        hardware.elevatorRetainer().setPosition(ELEVATOR_CLOSE);
    }

    @Override
    public void start() {
        hardware.getBallScorer().startTeleop();
        hardware.ballHolderServo().setPosition(0.5);
    }

    @Override
    public void loop() {
        double[] motors = calculateMotorValues(-gamepad1.left_stick_x, -gamepad1.left_stick_y,
                -gamepad1.right_stick_x, -gamepad1.right_stick_y);

        hardware.getFrontLeftMotor().setPower(motors[FRONT_LEFT]);
        hardware.getFrontRightMotor().setPower(motors[FRONT_RIGHT]);
        hardware.getBackLeftMotor().setPower(motors[BACK_LEFT]);
        hardware.getBackRightMotor().setPower(motors[BACK_RIGHT]);


        // Intake
        if (gamepad1.left_trigger > 0.5 || gamepad2.x)
            // Out
            hardware.getIntakeMotor().setPower(-1);
        else if (gamepad1.right_trigger > 0.5 || gamepad2.a)
            // In
            hardware.getIntakeMotor().setPower(1);
        else
            hardware.getIntakeMotor().setPower(0);

        updateElevator();
        elevatorClamp();

        if (gamepad1.x || (gamepad2.back || gamepad2.left_bumper || gamepad2.right_bumper)) {
            hardware.getBallScorer().launch();
        }

        if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5) {
            hardware.getBallScorer().home();
        }

        hardware.semaphore().setPosition(hardware.getBallScorer().getState() == BallScorer.State.WAITING ? 1 : 0);
        motorUpdate();
    }

    private void elevatorClamp() {
        if (gamepad1.b || gamepad2.dpad_right) {
            hardware.elevatorRetainer().setPosition(ELEVATOR_OPEN);
        } else if (gamepad1.y || gamepad2.dpad_left) {
            hardware.elevatorRetainer().setPosition(ELEVATOR_CLOSE);
        }
    }

    private void motorUpdate() {
        final double scaleFactor = 0.25;
        DcMotor frontLeft = hardware.getFrontLeftMotor();
        DcMotor frontRight = hardware.getFrontRightMotor();
        DcMotor backLeft = hardware.getBackLeftMotor();
        DcMotor backRight = hardware.getBackRightMotor();

        if (Math.abs(hardware.getElevatorMotor().getCurrentPosition()) > 20000) {
            frontLeft.setPower(frontLeft.getPower() * scaleFactor);
            frontRight.setPower(frontRight.getPower() * scaleFactor);
            backLeft.setPower(backLeft.getPower() * scaleFactor);
            backRight.setPower(backRight.getPower() * scaleFactor);
        }
    }

    private void updateElevator() {
        int DELTA_CHANGE = 200;
        if (gamepad1.right_bumper || gamepad2.dpad_up) {
            hardware.getElevatorMotor().setPower(1);
            hardware.getElevatorMotor().setTargetPosition(hardware.getElevatorMotor().getCurrentPosition() + DELTA_CHANGE);
        }
        if (gamepad1.left_bumper || gamepad2.dpad_down) {
            hardware.getElevatorMotor().setPower(1);
            hardware.getElevatorMotor().setTargetPosition(hardware.getElevatorMotor().getCurrentPosition() - DELTA_CHANGE);
        }
    }

    private double[] calculateMotorValues(float joyLX, float joyLY, float joyRX, float joyRY) {
        double[] motorPower = new double[]{0, 0, 0, 0};
        joyRY = scaleInput(joyRY);
        joyLY = scaleInput(joyLY);
        joyRX = scaleInput(joyRX);
        joyLX = scaleInput(joyLX);

        float modX = -(joyLX + joyRX) / 2;
        telemetry.addData("modX: ", modX);

        motorPower[FRONT_LEFT] = modX + joyLY;
        motorPower[FRONT_RIGHT] = joyRY - modX;
        motorPower[BACK_LEFT] = joyLY - modX;
        motorPower[BACK_RIGHT] = modX + joyRY;

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
}
