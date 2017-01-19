package org.tka.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.tka.robotics.utils.BallScorer;

@TeleOp(name = "Ball Launch Test")
public class BallLauncherTest extends OpMode {

    private BallScorer scorer;

    private Thread ballScorerThread;


    @Override
    public void init() {
        scorer = new BallScorer(this, hardwareMap.dcMotor.get("motor"),
                hardwareMap.crservo.get("servo"), hardwareMap.touchSensor.get("touch_sensor"));
        ballScorerThread = new Thread(scorer);
        ballScorerThread.setDaemon(true);
        ballScorerThread.setName("BallScorerThread");
    }

    @Override
    public void start() {
        ballScorerThread.start();
    }

    @Override
    public void loop() {
        if (gamepad1.x && scorer.getState() == BallScorer.State.WAITING) {
            scorer.LAUNCHDATHING();
        }
        telemetry.addData("State", scorer.getState());
        telemetry.update();
    }

    @Override
    public void stop() {
        scorer.stop();
    }
}
