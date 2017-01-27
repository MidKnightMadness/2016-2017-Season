package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.tka.robotics.utils.hardware.MainBotHardware;

/**
 * Created by Joshua on 10/15/2016.
 */

@Autonomous(name="PANIC test")
public class Test extends LinearOpMode {

    private MainBotHardware hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new MainBotHardware(this);

        waitForStart();


        hardware.getUtilities().resetDriveMotors();

//        hardware.getUtilities().driveForward(1500, 0.4);
//
//        hardware.getUtilities().backwardLeftDiagonal(5000, 0.4);

        while(true) {
            hardware.getFrontRightMotor().setPower(1);
            hardware.getBackRightMotor().setPower(0.3);
        }



    }
}