package org.tka.robotics.utils.pid;

import android.util.Log;
import android.widget.TextView;

import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Iterator;

public class PidUpdater extends Thread {

    private static final String TAG = "PID-Updater";

    private static final Object motorLock = new Object();
    public static PidUpdater INSTANCE;
    public static boolean running = true;
    // A Singleton instance of the PID updater thread
    private HardwareMap hardwareMap;
    private ArrayList<PidMotor> motors = new ArrayList<>();
    private boolean tornDown = true;

    private Field updateUIField, robotState;

    public PidUpdater(HardwareMap map) {
        running = true;
        this.hardwareMap = map;
        setName("PIDUpdater");
        setDaemon(true);
        start();
    }

    public static void shutdown() {
        running = false;
    }

    protected void registerMotor(PidMotor motor) {
        synchronized (motorLock) {
            this.motors.add(motor);
            motor.init();
            motor.update();
        }
    }

    @Override
    public void run() {
        Log.i(TAG, "Initializing PID Updater");
        while (true) {
            // Get the running op mode
            String rop = runningOpMode();
            // Check if the op mode equals 'Stop Robot'
            // NOTE: This is a terrible hack, which may or may not work. Use at your own risk.
            if (!rop.equalsIgnoreCase("Stop Robot")) {
                tornDown = false;
                // Acquire an Intrinsic Lock (google it) on the motors
                synchronized (motorLock) {
                    // Update all the motors
                    for (PidMotor m : motors) {
                        m.update();
                    }
                }
            } else {
                // If the op mode is 'Stop Robot', tear down the motors
                synchronized (motorLock) {
                    if (!tornDown) {
                        Log.i(TAG, "Beginning teardown");
                        // Iterate through all the motors and tear them down
                        Iterator<PidMotor> iterator = motors.iterator();
                        while (iterator.hasNext()) {
                            PidMotor m = iterator.next();
                            Log.i(TAG, "Tearing down " + m.getName());
                            // Set the motor's velocity to 0
                            m.setVelocity(0);
                            // Set the motor power to zero, stopping the motor
                            m.getMotor().setPower(0);
                            // Remove the motor from the array
                            iterator.remove();
                        }
                        Log.i(TAG, "Teardown Complete!");
                        tornDown = true;
                    }
                }
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Thread.yield();
        }
    }

    /**
     * <b>WARNING!!! ADVANCED CODING AHEAD!</b>
     * <br/>
     * Get the current op mode that is running
     *
     * @return The op mode that is running
     */
    private String runningOpMode() {
        try {
            if (updateUIField == null) {
                updateUIField = FtcRobotControllerActivity.class.getDeclaredField("updateUI");
                updateUIField.setAccessible(true);
            }
            UpdateUI ui = (UpdateUI) updateUIField.get(hardwareMap.appContext);
            if (robotState == null) {
                robotState = UpdateUI.class.getDeclaredField("textOpMode");
                robotState.setAccessible(true);
            }
            return ((TextView) robotState.get(ui)).getText().toString().substring(9);
        } catch (IllegalAccessException | NoSuchFieldException e) {
            e.printStackTrace();
        }
        return "Stop Robot";
    }
}
