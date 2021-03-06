package org.tka.robotics.utils.vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

import java.util.Locale;

@Autonomous(name = "FTCVuforiaTest")
public class FtcVuforiaTest extends OpMode {

    private FtcVuforia vuforia;

    @Override
    public void init() {
       vuforia =  new FtcVuforia(R.id.cameraMonitorViewId, VuforiaLocalizer.CameraDirection.FRONT);
        vuforia.setTargets(FtcVuforia.locationMatrix(0, 0, 0, 6 * FtcVuforia.MM_PER_INCH, 0, 0));
    }

    @Override
    public void start() {
        vuforia.setTrackingEnabled(true);
    }

    @Override
    public void loop() {
        vuforia.updateRobotLocation();
        float[] pos = vuforia.getRobotPosition();
        if(pos == null){
            telemetry.addData("Robot Pos", "Unknown");
        } else {
            String position = String.format(Locale.ENGLISH, "%.2f, %.2f, %.2f", pos[0], pos[1], pos[2]);
            telemetry.addData("Robot Pos: ", position);
        }
        float[] rot = vuforia.getRobotOrientation();
        if(rot == null){
            telemetry.addData("Robot Rot", "Unknown");
        } else {
            String rotation = String.format(Locale.ENGLISH, "%.2f, %,2f %.2f", rot[0], rot[1], rot[2]);
            telemetry.addData("Robot Rot", rotation);
        }
        if(vuforia.getLastKnownLocation() != null)
        telemetry.addData("Location", vuforia.getLastKnownLocation().formatAsTransform());
        else
        telemetry.addData("Location", "Unknown");
        telemetry.update();
    }
}
