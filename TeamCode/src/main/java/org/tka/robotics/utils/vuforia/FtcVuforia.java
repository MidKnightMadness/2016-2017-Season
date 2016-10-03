package org.tka.robotics.utils.vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;

/**
 * Utility class to handle all the "dirty" work of using Vuforia for locating the robot
 */
public class FtcVuforia {

    // Declare constants
    public static final float MM_PER_INCH = 25.4f;
    public static final float FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;
    private static final String VUFORIA_LICENSE_KEY = "AcF4Fen/////AAAAGRpWPEA6DkVapPT8h4D5ABozK1RZ3YsKUwHZzIVVWPzksL3r3" +
            "rHDhFZNXqWXupugaYnNZHy95Pf+vKqje93szcW6OCpJ4H66mtHrVuirEB4gAsVglOLqgaMkHWms0p4dxduw" +
            "Zc+HzxGTmuF7oPWuX57Bg/4TesNeD0vgT1X1M2tedf0F4U6bavaEtis+5In/7/fBJ1n6Fv/qLcHPtxAJ+nM" +
            "zFycCv4HkEQhnhOxcT8TnTleAAFEirS7gP6jufZ3Sy7JrYQBLkMs+BstrhjdIGOGLKbohyiLJ9/EWl8udGn" +
            "v64zo58l5QWLhDzpCNgsJO6suEPX9U3Z9TD9Aa2cdNEYjjGCKz8TG5j61YhSjpzNDw";
    // Declare default targets (2016-2017 Season)
    private static final Target[] DEFAULT_TARGETS = new Target[]{
            // Blue Alliance beacon 1
            new Target("wheels", 90.0F, 0.0F, 0.0F, 12.0F * MM_PER_INCH, FIELD_WIDTH / 2.0F, 160),
            // Red Alliance beacon 2
            new Target("tools", 90.0F, 0.0F, 90.0F, -FIELD_WIDTH / 2.0F, 30.0F * MM_PER_INCH, 160),
            // Blue Alliance beacon 2
            new Target("legos", 90.0F, 0.0F, 0.0F, -30.0F * MM_PER_INCH, FIELD_WIDTH / 2.0F, 160),
            // Red Alliance beacon 1
            new Target("gears", 90.0F, 0.0F, 90.0F, -FIELD_WIDTH / 2.0F, -12.0F * MM_PER_INCH, 160)
    };
    private VuforiaLocalizer.Parameters parameters;
    // A list of targets loaded from the asset file
    private VuforiaTrackables targetList;
    // A list of all the targets that should be tracked
    private Target[] targets;

    // Keep track of the known location
    private OpenGLMatrix lastKnownLocation;
    private boolean trackingEnabled = false;

    /**
     * Creates an instance of this object. It initializes Vuforia with the camera view's ID, asset file, as well as
     * direction.
     *
     * @param cameraId        The camera view id
     * @param assetFile       The asset file to load the trackables from
     * @param cameraDirection The camera to use on the phone (front or back)
     */
    public FtcVuforia(int cameraId, String assetFile, VuforiaLocalizer.CameraDirection cameraDirection) {
        // Set up essential Vuforia stuff, such as the preview id, license key, and direction
        this.parameters = new VuforiaLocalizer.Parameters(cameraId);
        this.parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        this.parameters.cameraDirection = cameraDirection;
        VuforiaLocalizer localizer = ClassFactory.createVuforiaLocalizer(parameters);
        // Load the trackable assets
        this.targetList = localizer.loadTrackablesFromAsset(assetFile);
    }


    /**
     * Creates an instance of this object, defaulting to using the asset file "FTC_2016-17"
     *
     * @param cameraId  The camera view id
     * @param direction The camera to use on the phone (Front or back)
     */
    public FtcVuforia(int cameraId, VuforiaLocalizer.CameraDirection direction) {
        this(cameraId, "FTC_2016-17", direction);
    }

    /**
     * Constructs an {@link OpenGLMatrix} from the given parameters, both translating and rotating the matrix.
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
        // First, translate the matrix in the X, Y, and Z directions
        return OpenGLMatrix.translation(posX, posY, posZ)
                // Multiply by a rotation matrix, because multiplying translation matrices combines their effect
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, XYZ, AngleUnit.DEGREES, rotX, rotY, rotZ));
    }

    /**
     * Gets the robot's X, Y and Z position. Usually, X and Y will be used
     *
     * @return A float array of coordinates ordered by X, Y, then Z
     */
    public float[] getRobotPosition() {
        updateRobotLocation();
        float[] pos = new float[3];
        if (lastKnownLocation == null)
            return pos;
        VectorF translation = lastKnownLocation.getTranslation();
        pos[0] = translation.get(0);
        pos[1] = translation.get(1);
        pos[2] = translation.get(2);
        return pos;
    }

    /**
     * Gets a {@link VuforiaTrackable} target by its name
     *
     * @param name The name of the target to get
     * @return The target, or null if it doesn't exist
     */
    public VuforiaTrackable getTarget(String name) {
        for (VuforiaTrackable t : targetList) {
            if (t.getName().equalsIgnoreCase(name))
                return t;
        }
        return null;
    }

    /**
     * Gets a {@link VuforiaTrackable} target by its index in the trackables file
     *
     * @param index The index of the target
     * @return The target, or null if it doesn't exist
     */
    public VuforiaTrackable getTarget(int index) {
        return targetList.get(index);
    }

    /**
     * Gets an {@link OpenGLMatrix} representing the <b>ROBOT'S</b> location in reference to the given target
     *
     * @param target The target
     * @return The robot's location, or null, if the target isn't visible
     */
    public OpenGLMatrix getTargetLocation(VuforiaTrackable target) {
        return ((VuforiaTrackableDefaultListener) target.getListener()).getRobotLocation();
    }

    /**
     * Gets an {@link OpenGLMatrix} representing the <b>ROBOT'S</b> location in reference to the given target
     *
     * @param target The target
     * @return The robot's location, or null, if the target isn't visible
     */
    public OpenGLMatrix getTargetLocation(Target target) {
        return getTargetLocation(target.trackable);
    }

    /**
     * Gets an {@link OpenGLMatrix} representing the <b>ROBOT'S</b> pose in reference to the given target
     *
     * @param target The target
     * @return The robot's pose, or null, if the target isn't visible
     */
    public OpenGLMatrix getTargetPose(VuforiaTrackable target) {
        return ((VuforiaTrackableDefaultListener) target.getListener()).getPose();
    }

    /**
     * Gets an {@link OpenGLMatrix} representing the <b>ROBOT'S</b> pose in reference to the given target
     *
     * @param target The target
     * @return The robot's pose, or null, if the target isn't visible
     */
    public OpenGLMatrix getTargetPose(Target target) {
        return getTargetPose(target.trackable);
    }

    /**
     * Gets a list of all targets that are currently visible
     *
     * @return A {@link ArrayList} of all visible targets
     */
    public List<Target> getVisibleTargets() {
        List<Target> visibleTargets = new ArrayList<>();
        for (Target t : targets) {
            if (((VuforiaTrackableDefaultListener) t.trackable.getListener()).isVisible()) {
                visibleTargets.add(t);
            }
        }
        return visibleTargets;
    }

    /**
     * Sets a target's information such as its name, location on the field, and the phone's location on the robot
     *
     * @param t               The {@link Target} to set information for
     * @param index           The index in the asset file of the target
     * @param name            The name to use for the target
     * @param locationOnField The location on the field of the target
     * @param phoneLocOnRobot The location of the phone on thte robot
     */
    public void setTargetInfo(Target t, int index, String name, OpenGLMatrix locationOnField, OpenGLMatrix phoneLocOnRobot) {
        VuforiaTrackable target = targetList.get(index);
        target.setName(name);
        if (locationOnField != null)
            target.setLocation(locationOnField);
        if (phoneLocOnRobot != null)
            ((VuforiaTrackableDefaultListener) target.getListener()).setPhoneInformation(phoneLocOnRobot, this.parameters.cameraDirection);
        t.trackable = target;
    }

    /**
     * Sets information about the given targets
     *
     * @param targets              An array of targets (Must be in the same order as the asset file)
     * @param phoneLocationOnRobot The phone's location on the robot
     */
    public void setTargets(Target[] targets, OpenGLMatrix phoneLocationOnRobot) {
        this.targets = targets;
        for (int i = 0; i < targets.length; i++) {
            Target target = targets[i];
            OpenGLMatrix targetLoc = locationMatrix(target.rotX, target.rotY, target.rotZ, target.posX,
                    target.posY, target.posZ);
            setTargetInfo(targets[i], i, target.name, targetLoc, phoneLocationOnRobot);
        }
    }

    /**
     * Uses the default target (in this case, for the 2016-2017 season) information
     *
     * @param phoneLocOnRobot The phone's location on the robot
     */
    public void setTargets(OpenGLMatrix phoneLocOnRobot) {
        setTargets(DEFAULT_TARGETS, phoneLocOnRobot);
    }

    /**
     * Enable or disable the tracking of the targets
     *
     * @param enabled Weather to enable or disable targets
     */
    public void setTrackingEnabled(boolean enabled) {
        trackingEnabled = enabled;
        if (enabled) {
            targetList.activate();
        } else {
            targetList.deactivate();
        }
    }

    /**
     * Loops through all visible targets and gets the location from them
     */
    public void updateRobotLocation() {
        if (!trackingEnabled)
            throw new IllegalStateException("Cannot get the robot's location if tracking isn't enabled");
        // Get a list of all targets that are currently visible
        List<Target> visibleTargets = this.getVisibleTargets();
        // Loop through all the targets
        for (Target t : visibleTargets) {
            // If the target is visible, set the last known location to the current location
            if (getTargetLocation(t) != null) {
                lastKnownLocation = getTargetLocation(t);
            }
        }
    }

    /**
     * Class representing information about a target's location and rotation
     */
    public static class Target {
        public final String name;
        private final float rotX, rotY, rotZ;
        private final float posX, posY, posZ;
        private VuforiaTrackable trackable;

        public Target(String name, float posZ, float posY, float posX, float rotZ, float rotY, float rotX) {
            this.name = name;
            this.posZ = posZ;
            this.posY = posY;
            this.posX = posX;
            this.rotZ = rotZ;
            this.rotY = rotY;
            this.rotX = rotX;
        }
    }
}
