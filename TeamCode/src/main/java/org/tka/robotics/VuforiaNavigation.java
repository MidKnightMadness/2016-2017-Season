package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Vuforia")
public class VuforiaNavigation extends LinearOpMode {

    private final String TAG = "VLOC";
    private OpenGLMatrix lastPosition = null;

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Vuforia
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey= "AcF4Fen/////AAAAGRpWPEA6DkVapPT8h4D5ABozK1RZ3YsKUwHZzIVVWPzksL3r3" +
                "rHDhFZNXqWXupugaYnNZHy95Pf+vKqje93szcW6OCpJ4H66mtHrVuirEB4gAsVglOLqgaMkHWms0p4dxduw" +
                "Zc+HzxGTmuF7oPWuX57Bg/4TesNeD0vgT1X1M2tedf0F4U6bavaEtis+5In/7/fBJ1n6Fv/qLcHPtxAJ+nM" +
                "zFycCv4HkEQhnhOxcT8TnTleAAFEirS7gP6jufZ3Sy7JrYQBLkMs+BstrhjdIGOGLKbohyiLJ9/EWl8udGn" +
                "v64zo58l5QWLhDzpCNgsJO6suEPX9U3Z9TD9Aa2cdNEYjjGCKz8TG5j61YhSjpzNDw";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(params);

        // Load tracking assets
        VuforiaTrackables  trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable wheels = trackables.get(0);
        VuforiaTrackable tools = trackables.get(1);
        VuforiaTrackable legos = trackables.get(2);
        VuforiaTrackable gears = trackables.get(3);

        List<VuforiaTrackable> allTrackables = new ArrayList<>();
        allTrackables.add(wheels);
        allTrackables.add(tools);
        allTrackables.add(legos);
        allTrackables.add(gears);


        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;
        float mmFieldWidth = (12*12 - 2) * mmPerInch; // The field is 12', but the walls are 1", giving us 11'10" of usable space

        OpenGLMatrix wheelsLocationOnField = OpenGLMatrix.translation((18*3)*mmPerInch, 0, 0)
                .multiplied(OpenGLMatrix.rotation(AxesReference.EXTRINSIC,
                        AxesOrder.XYX, AngleUnit.DEGREES, 180, 0, 0));
        wheels.setLocation(wheelsLocationOnField);
        RobotLog.ii(TAG, "wheels = %s", format(wheelsLocationOnField));

        OpenGLMatrix legosLocationOnField = OpenGLMatrix.translation((5*18+13) * mmPerInch, 0, 0)
                .multiplied(OpenGLMatrix.rotation(AxesReference.EXTRINSIC,
                        AxesOrder.XYX, AngleUnit.DEGREES, 180, 0, 0));
        legos.setLocation(legosLocationOnField);
        RobotLog.ii(TAG, "legos = %s", format(legosLocationOnField));

        OpenGLMatrix phoneLocOnBot = OpenGLMatrix.translation(12 * mmPerInch, 0, 0);
        RobotLog.ii(TAG, "phone = %s", format(phoneLocOnBot));
        ((VuforiaTrackableDefaultListener) wheels.getListener()).setPhoneInformation(phoneLocOnBot, params.cameraDirection);
        ((VuforiaTrackableDefaultListener) legos.getListener()).setPhoneInformation(phoneLocOnBot, params.cameraDirection);
        telemetry.addData(">", "Press play to start");
        telemetry.update();

        waitForStart();

        trackables.activate();

        while(opModeIsActive()){
            for(VuforiaTrackable t : allTrackables){
                VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) t.getListener();
                telemetry.addData(t == wheels? "Wheels" : "Legos", listener.isVisible()? "Visible" : "Not Visible");
                OpenGLMatrix matrix = listener.getUpdatedRobotLocation();
                if(matrix != null){
                    lastPosition = matrix;
                }
            }
            telemetry.addData("Pos ", lastPosition == null? "Unknown" : format(lastPosition));
            telemetry.update();
            idle();
        }
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    private String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
