package org.tka.robotics;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import org.tka.robotics.utils.hardware.MainBotHardware;

@Autonomous(name = "ITS TIME TO DRIFT")
public class DriftTest extends LinearOpMode{

    private MainBotHardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new MainBotHardware(this);


        tunePid();
        hardware.getUtilities().resetDriveMotors();
        waitForStart();

        hardware.getFrontRightMotor().setPower(0.8);
        hardware.getBackLeftMotor().setPower(0.8);
        hardware.getFrontLeftMotor().setPower(0);
        hardware.getBackRightMotor().setPower(0);

        long stopTime = System.currentTimeMillis() + 5000;
        while(stopTime > System.currentTimeMillis()){
            telemetry.update();
            idle();
        }

        hardware.stopAllMotors();

        while(opModeIsActive()){
            telemetry.update();
            idle();
        }
    }

    private void tunePid(){
        ModernRoboticsUsbDcMotorController leftMotors = (ModernRoboticsUsbDcMotorController) hardware.getFrontLeftMotor().getController();
        ModernRoboticsUsbDcMotorController rightMotors = (ModernRoboticsUsbDcMotorController) hardware.getBackRightMotor().getController();

        DifferentialControlLoopCoefficients coeff = new DifferentialControlLoopCoefficients(0xE0, 0x40, 0x70);
        leftMotors.setDifferentialControlLoopCoefficients(1, coeff);
        leftMotors.setDifferentialControlLoopCoefficients(2, coeff);

        rightMotors.setDifferentialControlLoopCoefficients(1, coeff);
        rightMotors.setDifferentialControlLoopCoefficients(2, coeff);
    }
}
