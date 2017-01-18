package org.tka.robotics.utils.vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper class to abstract out vision tracking code
 */
public class FtcVuforia {

    // Constants used throughout
    public static final float MM_PER_INCH = 25.4f;
    public static final float FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;
    public static final Target[] DEFAULT_TARGETS = new Target[]{
            new Target("wheels", 12.0F * MM_PER_INCH, FIELD_WIDTH / 2.0F, 160).rotate(90.0F, 0.0F, 0.0F),
            new Target("tools", -FIELD_WIDTH / 2.0F, 30.0F * MM_PER_INCH, 160).rotate(90.0F, 0.0F, 90.0F),
            new Target("legos", -30.0F * MM_PER_INCH, FIELD_WIDTH / 2.0F, 160).rotate(90.0F, 0.0F, 0.0F),
            new Target("gears", -FIELD_WIDTH / 2.0F, -12.0F * MM_PER_INCH, 160).rotate(90.0F, 0.0F, 90.0F)
    };
    private static final String LICENSE_KEY = "AcF4Fen/////AAAAGRpWPEA6DkVapPT8h4D5ABozK1RZ3YsKUwHZzIVVWPzksL3r3" +
            "rHDhFZNXqWXupugaYnNZHy95Pf+vKqje93szcW6OCpJ4H66mtHrVuirEB4gAsVglOLqgaMkHWms0p4dxduw" +
            "Zc+HzxGTmuF7oPWuX57Bg/4TesNeD0vgT1X1M2tedf0F4U6bavaEtis+5In/7/fBJ1n6Fv/qLcHPtxAJ+nM" +
            "zFycCv4HkEQhnhOxcT8TnTleAAFEirS7gP6jufZ3Sy7JrYQBLkMs+BstrhjdIGOGLKbohyiLJ9/EWl8udGn" +
            "v64zo58l5QWLhDzpCNgsJO6suEPX9U3Z9TD9Aa2cdNEYjjGCKz8TG5j61YhSjpzNDw";
    private static final String DEFAULT_TRACKABLES_FILE = "FTC_2016-17";

    // Internal vuforia stuff (parameters, trackable list, and target)
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables trackables;
    private Target[] targets;
    private boolean trackingEnabled = false;

    // Contains the last known robot location
    private OpenGLMatrix lastKnownLocation;
    private long lastKnownAt;
    private Target lastSeenTarget;

    private Thread updaterThread;


    /**
     * Creates an instance of this object. This initializes Vuforia with the camera view's ID (the preview window), asset file,
     * as well as which camera to use
     *
     * @param cameraId        The view id for the preview
     * @param assetFile       The asset file to load
     * @param cameraDirection The camera direction
     */
    public FtcVuforia(int cameraId, String assetFile, VuforiaLocalizer.CameraDirection cameraDirection) {
        // Set up essential Vuforia stuff, such as the preview id, license key, and direction
        this.parameters = new VuforiaLocalizer.Parameters(cameraId);
        this.parameters.vuforiaLicenseKey = LICENSE_KEY;
        this.parameters.cameraDirection = cameraDirection;
        VuforiaLocalizer localizer = ClassFactory.createVuforiaLocalizer(parameters);
        // Load the trackable assets
        this.trackables = localizer.loadTrackablesFromAsset(assetFile);

        this.updaterThread = new Thread(new Updater());
        this.updaterThread.setName("VuforiaUpdater");
        this.updaterThread.setDaemon(true);
        this.updaterThread.start();
    }

    /**
     * Creates an instance of this object. This initializes Vuforia with the camera view's ID (For previewing) and which camera
     * to use. Defaults to using the 2016-2017 tracking code
     *
     * @param cameraId  The view id for the preview
     * @param direction The camera direction
     */
    public FtcVuforia(int cameraId, VuforiaLocalizer.CameraDirection direction) {
        this(cameraId, DEFAULT_TRACKABLES_FILE, direction);
    }

    /**
     * Coonstructs an {@link OpenGLMatrix} by first creating a translation matrix and rotation matrix, and multiplying the two.
     * <p>
     * In FTC, the reference frame used is the wall in front of the Red alliance's driver station, facing the field.
     * <ul>
     * <li>The X-axis runs parallel to the wall, with the positive X axis on the right.</li>
     * <li>The Y-axis runs perpendicular to the wall, and increases going away from the driver station.</li>
     * <li>The origin (0, 0, 0) is the point in the exact center of the field, just on top of the tiles</li>
     * </ul>
     * </p>
     *
     * @param rotX The rotation (in degrees) in the X axis
     * @param rotY The rotation (in degrees) in the Y axis
     * @param rotZ The rotation (in degrees) in the Z axis
     * @param posX The translation from the origin in the X direction
     * @param posY The translation from the origin in the Y direction
     * @param posZ The translation from the origin in the Z direction
     * @return An {@link OpenGLMatrix} representing the passed in values
     */
    public static OpenGLMatrix locationMatrix(float rotX, float rotY, float rotZ, float posX, float posY, float posZ) {
        return OpenGLMatrix.translation(posX, posY, posZ)
                // Multiply because multiplication combines the two translation matrices
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, rotX, rotY, rotZ));
    }

    /**
     * Returns the target that was last seen
     *
     * @return The last seen {@link Target}
     */
    public Target getLastSeenTarget() {
        return lastSeenTarget;
    }

    /**
     * Gets the robot's location using a {@link VuforiaTrackable} as the target
     *
     * @param target The target
     * @return An {@link OpenGLMatrix} of the robot's location
     */
    public OpenGLMatrix getRobotPosition(VuforiaTrackable target) {
        return getListener(target).getRobotLocation();
    }

    /**
     * Gets the robot's location using a {@link Target} as the target
     *
     * @param target The target
     * @return An {@link OpenGLMatrix} of the robot's location
     */
    public OpenGLMatrix getRobotPosition(Target target) {
        return this.getRobotPosition(target.trackable);
    }

    /**
     * Gets the robot's location on the field.
     *
     * @return A float array with the following indexes:
     * <ul>
     * <li><strong>0: </strong>X position (in mm)</li>
     * <li><strong>1: </strong>Y position (in mm)</li>
     * <li><strong>2: </strong>Z position (in mm)</li>
     * </ul>
     */
    public float[] getRobotPosition() {
//        updateRobotLocation();
        if (lastKnownLocation == null) {
            return null;
        }


        return lastKnownLocation.getTranslation().getData();
    }

    /**
     * Gets the last known robot location
     *
     * @return The last known location
     */
    public OpenGLMatrix getLastKnownLocation() {
        return lastKnownLocation;
    }

    /**
     * Gets the robot's rotation on the field
     *
     * @return A float array with the following indexes:
     * <ul>
     * <li><strong>0: </strong>X rotation (degrees)</li>
     * <li><strong>1: </strong>Y rotation (degrees)</li>
     * <li><strong>2: </strong>Z rotation (degrees)</li>
     * </ul>
     */
    public float[] getRobotOrientation() {
        if (lastKnownLocation == null)
            return null;

        Orientation orientation = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return new float[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};
    }

    /**
     * Gets a list of all visible targets
     *
     * @return An {@link ArrayList} of all the visible targets
     */
    public List<Target> getVisibleTargets() {
        List<Target> visible = new ArrayList<>();
        for (Target t : targets) {
            if (getListener(t.trackable).isVisible())
                visible.add(t);
        }
        return visible;
    }

    /**
     * Sets information about a target
     *
     * @param t               The target
     * @param index           The index (in the asset file) of the target
     * @param name            The name of the target
     * @param locationOnField The target's location on the field
     * @param phoneLocOnRobot The phone's location on the robot
     */
    public void setTargetInfo(Target t, int index, String name, OpenGLMatrix locationOnField, OpenGLMatrix phoneLocOnRobot) {
        VuforiaTrackable target = trackables.get(index);
        target.setName(name);
        if (locationOnField != null)
            target.setLocation(locationOnField);
        if (phoneLocOnRobot != null) {
            getListener(target).setPhoneInformation(phoneLocOnRobot, this.parameters.cameraDirection);
        }
        t.trackable = target;
    }

    /**
     * Sets the targets to use for vision tracking
     *
     * @param targets              The target array to use. <strong>NOTE:</strong> Must be in the same order as the asset file
     * @param phoneLocationOnRobot The phone's location on the robot
     */
    public void setTargets(Target[] targets, OpenGLMatrix phoneLocationOnRobot) {
        this.targets = targets;
        for (int i = 0; i < targets.length; i++) {
            Target target = targets[i];
            OpenGLMatrix targetLoc = locationMatrix(target.rotX, target.rotY, target.rotZ, target.posX, target.posY, target.posZ);
            setTargetInfo(target, i, target.name, targetLoc, phoneLocationOnRobot);
        }
    }

    /**
     * Uses thte default vision tracking gargets
     *
     * @param phoneLocationOnRobot The phone's location on the robot
     */
    public void setTargets(OpenGLMatrix phoneLocationOnRobot) {
        setTargets(DEFAULT_TARGETS, phoneLocationOnRobot);
    }

    /**
     * Enables or disables tracking
     *
     * @param enabled True to enable, false to disable
     */
    public void setTrackingEnabled(boolean enabled) {
        trackingEnabled = enabled;
        if (enabled)
            trackables.activate();
        else
            trackables.deactivate();
    }

    /**
     * Updates the robot's location
     */
    public void updateRobotLocation() {
        if (!trackingEnabled)
            throw new IllegalStateException("Cannot get the robot's location if tracking isn't enabled!");
        List<Target> visibleTargets = this.getVisibleTargets();
        for (Target t : visibleTargets) {
            OpenGLMatrix location = (getListener(t.trackable).getUpdatedRobotLocation());
            if (location != null) {
                lastKnownLocation = location;
                lastSeenTarget = t;
            }
        }
    }

    /**
     * Gets the {@link VuforiaTrackableDefaultListener} for a given {@link VuforiaTrackable}
     *
     * @param trackable The trackable
     * @return The listener
     */
    private VuforiaTrackableDefaultListener getListener(VuforiaTrackable trackable) {
        return (VuforiaTrackableDefaultListener) trackable.getListener();
    }

    /**
     * Represents a target on the field
     */
    public static class Target {
        private final String name;
        private final float posX, posY, posZ;
        private float rotX, rotY, rotZ;
        private VuforiaTrackable trackable;

        public Target(String name, float posX, float posY, float posZ) {
            this.name = name;
            this.posX = posX;
            this.posY = posY;
            this.posZ = posZ;
            // Mass assign the rotations to Zero
            this.rotX = this.rotY = this.rotZ = 0;
        }

        public String getName() {
            return name;
        }

        public Target rotate(float rotX, float rotY, float rotZ) {
            this.rotX = rotX;
            this.rotY = rotY;
            this.rotZ = rotZ;
            return this;
        }
    }

    private class Updater implements Runnable {

        private boolean running = true;

        @Override
        public void run() {
            while (running) {
                if (trackingEnabled)
                    updateRobotLocation();
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
