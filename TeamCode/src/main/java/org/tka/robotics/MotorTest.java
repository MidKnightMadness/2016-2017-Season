/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

/**
 * {@link MotorTest} illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry {@link Telemetry#log() log} is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
@Autonomous(name = "Motor Test", group = "Test")
@Disabled
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted()) {
            telemetry.addData("Status", "Waiting....");
            telemetry.update();
        }

        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(5);

        while (opModeIsActive()) {

            telemetry.log().add("Cycling through "+hardwareMap.dcMotor.size()+" motors");
            telemetry.update();


            ElapsedTime time = new ElapsedTime();
            time.reset();
            for (Map.Entry<String, DcMotor> m : hardwareMap.dcMotor.entrySet()) {
                DcMotor motor = m.getValue();
                String motorName = m.getKey();

                motor.resetDeviceConfigurationForOpMode();
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(0.3F);
                telemetry.log().add("Starting motor " + motorName);
                time.reset();
                while (time.seconds() < 4) {
                    telemetry.addData("Running motor", motorName);
                    telemetry.addData("Time Running", time.seconds());
                    telemetry.update();
                    idle();
                }
                telemetry.log().add("Stopping motor " + motorName);
                motor.setPower(0);
                time.reset();
                telemetry.log().add("Waiting for 3 seconds...");
                while(time.seconds() < 3){
                    telemetry.addData("Time Running", time.seconds());
                    telemetry.update();
                    idle();
                }
                idle();
            }
            telemetry.log().add("---------");
            telemetry.update();
            idle();
        }
    }
}
