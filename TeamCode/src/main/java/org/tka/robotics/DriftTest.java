package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import org.firstinspires.ftc.robotcore.external.Func;
import org.tka.robotics.utils.hardware.MainBotHardware;

@Autonomous(name = "ITS TIME TO DRIFT")
public class DriftTest extends LinearOpMode {

    private MainBotHardware hardware;
    private long startTime;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new MainBotHardware(this);


//        tunePid();
        hardware.getUtilities().resetDriveMotors();
        hardware.getUtilities().setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        startTime = System.currentTimeMillis();

        telemetry.addData("Current Position", new Func<Integer>() {
            @Override
            public Integer value() {
                return calculatePosition();
            }
        });

        hardware.getFrontRightMotor().setPower(1.0);
        hardware.getBackLeftMotor().setPower(1.0);
        hardware.getFrontLeftMotor().setPower(0);
        hardware.getBackRightMotor().setPower(0);

        while (hardware.getFrontRightMotor().getCurrentPosition() < 1500) {
            telemetry.update();
            idle();
        }

       /* while(this.calculatePosition() < 16000){
            hardware.getFrontRightMotor().setTargetPosition(this.calculatePosition());
            hardware.getBackLeftMotor().setTargetPosition(this.calculatePosition());
            telemetry.update();
            idle();
        }*/

        hardware.getFrontRightMotor().setPower(0);
        hardware.getBackRightMotor().setPower(0);
        hardware.getFrontLeftMotor().setPower(0);
        hardware.getBackLeftMotor().setPower(0);

        while (opModeIsActive()) {
            telemetry.update();
            idle();
        }

    }

    private int calculatePosition() {
        return (int) Math.round((System.currentTimeMillis() - startTime) * 0.6);
    }

    private void tunePid() {
        ModernRoboticsUsbDcMotorController leftMotors = (ModernRoboticsUsbDcMotorController) hardware.getFrontLeftMotor().getController();
        ModernRoboticsUsbDcMotorController rightMotors = (ModernRoboticsUsbDcMotorController) hardware.getBackRightMotor().getController();

        DifferentialControlLoopCoefficients coeff = new DifferentialControlLoopCoefficients(0xE0, 0x40, 0x70);
        leftMotors.setDifferentialControlLoopCoefficients(1, coeff);
        leftMotors.setDifferentialControlLoopCoefficients(2, coeff);

        rightMotors.setDifferentialControlLoopCoefficients(1, coeff);
        rightMotors.setDifferentialControlLoopCoefficients(2, coeff);
    }
}
