package org.tka.robotics.utils.vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(name = "FTCVuforiaTest")
public class FtcVuforiaTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcVuforia vuforia = new FtcVuforia(R.id.cameraMonitorViewId, VuforiaLocalizer.CameraDirection.BACK);
        vuforia.setTargets(FtcVuforia.locationMatrix(0, 0, 0, 6 * FtcVuforia.MM_PER_INCH, 0, 0));

        waitForStart();

        vuforia.setTrackingEnabled(true);

        while (opModeIsActive()) {
            vuforia.updateRobotLocation();
            float[] pos = vuforia.getRobotPosition();
            telemetry.addData("Robot Pos: ", "%.2F, %.2F, %.2F", pos[0], pos[1], pos[2]);
            telemetry.update();
            idle();
        }
    }
}
