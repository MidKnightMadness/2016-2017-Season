package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.tka.robotics.utils.hardware.MainBotHardware;
import org.tka.robotics.utils.hardware.RobotHardware;

/**
 * Created: 2-7-2017
 * @author Austin
 */
@Autonomous(name = "Delay Parking")
public class DelayPark extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new MainBotHardware(this);

        telemetry.addData("Status", "Initialized and ready!");
        telemetry.update();

        waitForStart();

        long waitUntil = System.currentTimeMillis() + 20 * 1000;
        while(System.currentTimeMillis() < waitUntil){
            telemetry.addData("Time left", (waitUntil - System.currentTimeMillis()) / 1000);
            telemetry.update();
            idle();
        }

        // Drive forward for 5 seconds
        robotHardware.getFrontLeftMotor().setPower(1);
        robotHardware.getFrontRightMotor().setPower(1);
        robotHardware.getBackLeftMotor().setPower(1);
        robotHardware.getBackRightMotor().setPower(1);

        waitUntil = System.currentTimeMillis() + 5 * 1000;
        while(System.currentTimeMillis() < waitUntil)
            idle();
        robotHardware.stopAllMotors();
    }
}
