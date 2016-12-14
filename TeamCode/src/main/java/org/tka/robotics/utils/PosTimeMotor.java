package org.tka.robotics.utils;

import android.widget.TextView;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A {@link DcMotor} for higher accuracy, translating commandedPower to a ticking encoder
 */
// TODO: 12/10/2016 Fix stopping motors doing bad things
public class PosTimeMotor implements DcMotor {

    private static final double ENCODER_CONSTANT = 2.7;
    public OpMode opMode;
    protected double commandedPower = 0;
    int offset = 0;
    private DcMotor motor;
    private long referenceTime;
    private boolean disabled = false;

    public PosTimeMotor(DcMotor motor) {
        this.motor = motor;
        initializePosTime();
    }

    @Override
    public void close() {
        motor.close();
    }

    public void disablePosTime() {
        disabled = true;
        motor.setMode(RunMode.RUN_USING_ENCODER);
    }

    public void enablePosTime() {
        disabled = false;
        initializePosTime();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public int getMaxSpeed() {
        return motor.getMaxSpeed();
    }

    @Override
    public void setMaxSpeed(int encoderTicksPerSecond) {
        // Do nothing un
        if (disabled)
            motor.getMaxSpeed();
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setMode(RunMode mode) {
        // Don't do anything if the PosTime motor is enabled
        if (disabled)
            motor.setMode(mode);
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public double getPower() {
        return commandedPower;
    }

    @Override
    public void setPower(double power) {
        if (disabled) {
            motor.setPower(power);
        } else {
            // Do some fancyness with our encoder tick thing
            if (power != this.commandedPower && power != 0) {
                referenceTime = System.currentTimeMillis();
                this.commandedPower = power;
                this.offset = motor.getCurrentPosition();
            }
            if(power == 0)
                this.commandedPower = 0;
            updatePower();
        }
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    public int getTarget() {
        long f = System.currentTimeMillis() - referenceTime;
        int i = (int) (Math.round(f) * (ENCODER_CONSTANT * commandedPower)) + offset;
        if(opMode != null) {
            opMode.telemetry.log().add("Pos: " + f);
            opMode.telemetry.log().add("i: " + i);
        }
        return i;
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public void setTargetPosition(int position) {
        // Do the power loop thing
        if (disabled)
            motor.setTargetPosition(position);
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    public void updatePower() {
        if (commandedPower != 0)
            motor.setTargetPosition(getTarget());
    }

    private void initializePosTime() {
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setPower(1F);
    }


    /**
     * Runnable to update the {@link PosTimeMotor PosTimeMotors}
     */
    public static class Updater implements Runnable {

        private List<PosTimeMotor> motorsToUpdate = new ArrayList<>();

        private boolean running = true;
        private HardwareMap map;
        private Field updateUIField, robotState;
        private static final Object motorLock = new Object();

        public Updater(HardwareMap map, PosTimeMotor... motors) {
            this.map = map;
            synchronized (motorLock) {
                Collections.addAll(motorsToUpdate, motors);
            }
        }

        public void addMotor(PosTimeMotor motor){
            synchronized (motorLock){
                motorsToUpdate.add(motor);
            }
        }

        @Override
        public void run() {
            while (running) {
                synchronized (motorLock) {
                    for (PosTimeMotor m : motorsToUpdate) {
                        m.updatePower();
                    }
                }
                if (runningOpMode().equalsIgnoreCase("stop robot"))
                    running = false;
                Thread.yield();
            }

            synchronized (motorLock) {
                for (PosTimeMotor m : motorsToUpdate) {
                    m.setPower(0);
                }
            }
        }

        public void stop() {
            running = false;
        }

        private String runningOpMode() {
            try {
                if (updateUIField == null) {
                    updateUIField = FtcRobotControllerActivity.class.getDeclaredField("updateUI");
                    updateUIField.setAccessible(true);
                }
                UpdateUI ui = (UpdateUI) updateUIField.get(map.appContext);
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
}
