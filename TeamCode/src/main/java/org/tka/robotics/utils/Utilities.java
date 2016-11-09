package org.tka.robotics.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.tka.robotics.utils.hardware.RobotHardware;

public class Utilities {

    private RobotHardware hardware;
    private OpMode parent;

    public Utilities(OpMode parent, RobotHardware hardware) {
        this.hardware = hardware;
        this.parent = parent;
    }

    public void turn(double power) {
        this.hardware.getFrontLeftMotor().setPower(power);
        this.hardware.getBackLeftMotor().setPower(power);
        this.hardware.getFrontRightMotor().setPower(-power);
        this.hardware.getBackRightMotor().setPower(-power);
    }

    public void driveForward(int distance, double power) throws InterruptedException {
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (distance > 0) {
            while (this.hardware.getFrontRightMotor().getCurrentPosition() < distance) {
                setAllMotors(power);
                Thread.yield();
            }
        } else {
            while (this.hardware.getFrontRightMotor().getCurrentPosition() > distance) {
                setAllMotors(-power);
                Thread.yield();
            }
        }

        this.hardware.stopAllMotors();
    }

    public void setAllMotors(double power) throws InterruptedException {
        this.hardware.getFrontLeftMotor().setPower(power);
        this.hardware.getFrontRightMotor().setPower(power);
        this.hardware.getBackLeftMotor().setPower(power);
        this.hardware.getBackRightMotor().setPower(power);
    }

    public void forwardLeftDiagonal(double distance, double power) throws InterruptedException {
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getBackLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBackLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFrontRightMotor().setPower(power);
        this.hardware.getBackLeftMotor().setPower(power);
        this.hardware.getFrontLeftMotor().setPower(0);
        this.hardware.getBackRightMotor().setPower(0);

        while (this.hardware.getFrontRightMotor().getCurrentPosition() < distance) {
            this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", this.hardware.getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            Thread.yield();
        }

        this.hardware.stopAllMotors();
    }

    public void forwardRightDiagonal(double distance, double power) throws InterruptedException {
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFrontRightMotor().setPower(0);
        this.hardware.getBackLeftMotor().setPower(0);
        this.hardware.getFrontLeftMotor().setPower(power);
        this.hardware.getBackRightMotor().setPower(power);

        while (this.hardware.getFrontLeftMotor().getCurrentPosition() < distance) {
            this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", this.hardware.getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            Thread.yield();
        }

        this.hardware.stopAllMotors();
    }


    public void backwardLeftDiagonal(double distance, double power) throws InterruptedException {
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getBackRightMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hardware.getFrontRightMotor().setPower(0);
        this.hardware.getBackLeftMotor().setPower(0);
        this.hardware.getFrontLeftMotor().setPower(-power);
        this.hardware.getBackRightMotor().setPower(-power);

        while (this.hardware.getFrontLeftMotor().getCurrentPosition() > -distance) {
            this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
            this.parent.telemetry.addData("back left", this.hardware.getBackLeftMotor().getCurrentPosition());
            this.parent.telemetry.update();
            Thread.yield();
        }

        this.hardware.stopAllMotors();
    }

    public void strafe(double distance, double power) throws InterruptedException {
        if (power < 0)
            throw new IllegalStateException("Power must be positive");

        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hardware.getFrontLeftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (distance > 0) {
            while (this.hardware.getFrontLeftMotor().getCurrentPosition() < distance) {
                this.hardware.getFrontLeftMotor().setPower(power);
                this.hardware.getFrontRightMotor().setPower(-power);
                this.hardware.getBackLeftMotor().setPower(-power);
                this.hardware.getBackRightMotor().setPower(power);
                this.parent.telemetry.addData("front left", this.hardware.getFrontLeftMotor().getCurrentPosition());
                this.parent.telemetry.update();
                Thread.yield();
            }
        } else {
            while (this.hardware.getFrontLeftMotor().getCurrentPosition() > distance) {
                this.hardware.getFrontLeftMotor().setPower(-power);
                this.hardware.getFrontRightMotor().setPower(power);
                this.hardware.getBackLeftMotor().setPower(power);
                this.hardware.getBackRightMotor().setPower(-power);
                this.parent.telemetry.addData("front right", this.hardware.getFrontRightMotor().getCurrentPosition());
                this.parent.telemetry.update();
                Thread.yield();
            }
        }

        this.hardware.stopAllMotors();
    }
}
