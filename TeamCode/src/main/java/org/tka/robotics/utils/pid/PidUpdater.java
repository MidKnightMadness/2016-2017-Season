package org.tka.robotics.utils.pid;

import java.util.ArrayList;

public class PidUpdater extends Thread{

    // A Singleton instance of the PID updater thread
    public static PidUpdater INSTANCE;

    private ArrayList<PidMotor> motors = new ArrayList<>();

    public static boolean running = true;

    public PidUpdater(){
        running = true;
        setName("PIDUpdater");
        setDaemon(true);
        start();
    }

    protected void registerMotor(PidMotor motor){
        this.motors.add(motor);
        motor.init();
        motor.update();
    }

    @Override
    public void run() {
        while(running){
            for(PidMotor m : motors){
                m.update();
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Thread.yield();
        }
        for(PidMotor m : motors){
            m.motor.setPower(0);
        }
    }

    public static void shutdown(){
        running = false;
        INSTANCE = null;
    }
}
