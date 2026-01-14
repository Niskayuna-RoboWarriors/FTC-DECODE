package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.frozenmilk.dairy.mercurial.ftc.Context;

public class MecanumDrive {
    Context ctx;
    MotorVelocityController fl, fr, bl, br;
    boolean headingLock = true;
    boolean lockOnZeroAngularVelocity = false;
    double angularVelocityLockingThreshold = .3;
    double angularVelocityLockingThresholdDirection = 0;
    double headingLockDirection = 0;

    double headingPGain = 6.5 * 0.6;
    double headingDGain = 0.28;
    ElapsedTime timer;

    public MecanumDrive(Context ctx) {
        this.ctx = ctx;
        DcMotorEx flm = ctx.hardwareMap().get(DcMotorEx.class, "frontLeft");
        DcMotorEx frm = ctx.hardwareMap().get(DcMotorEx.class, "frontRight");
        DcMotorEx blm = ctx.hardwareMap().get(DcMotorEx.class, "backLeft");
        DcMotorEx brm = ctx.hardwareMap().get(DcMotorEx.class, "backRight");
        flm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flm.setDirection(DcMotorEx.Direction.REVERSE);
        blm.setDirection(DcMotorEx.Direction.REVERSE);
        fl = new MotorVelocityController(flm, 1, 0);
        fr = new MotorVelocityController(frm, 1, 0);
        bl = new MotorVelocityController(blm, 1, 0);
        br = new MotorVelocityController(brm, 1, 0);
    }

    public void joystick(double angle, float angularVelocity) {
        double y = -ctx.gamepad1().left_stick_y;
        double x = -ctx.gamepad1().left_stick_x;
        double rx = ctx.gamepad1().right_stick_x;
        if (Math.abs(y) < 0.01) {
            y = 0;
        }
        if (Math.abs(x) < 0.01) {
            x = 0;
        }
        double botHeading = angle;
        if (botHeading < 0) {
            botHeading = 2 * Math.PI + botHeading;
        }
        if (Math.abs(rx) < 0.01) {
            rx = 0;
            if (!headingLock) {
                headingLock = true;
                lockOnZeroAngularVelocity = true;
                angularVelocityLockingThresholdDirection = -Math.signum(angularVelocity);
                headingLockDirection = botHeading;
            }
            if (lockOnZeroAngularVelocity) {
                if (Math.abs(angularVelocity) < angularVelocityLockingThreshold
                        || Math.signum(angularVelocity) == angularVelocityLockingThresholdDirection) {
                    headingLockDirection = botHeading;
                    lockOnZeroAngularVelocity = false;
                }
            }
        } else {
            headingLock = false;
        }

        if (headingLock) {
            double headingDifference = botHeading - headingLockDirection;
            if (headingDifference > Math.PI) {
                headingDifference = headingDifference - 2 * Math.PI;
            }
            if (headingDifference < -Math.PI) {
                headingDifference = 2 * Math.PI - headingDifference;
            }
            rx = headingPGain * headingDifference + headingDGain * angularVelocity;
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        fl.setTarget((rotY + rotX + rx) / denominator);
        bl.setTarget((rotY - rotX + rx) / denominator);
        fr.setTarget((rotY - rotX - rx) / denominator);
        br.setTarget((rotY + rotX - rx) / denominator);
    }
    public void enableHeadingLock() {
        headingLock = true;
    }
    public void disableHeadingLock() {
        headingLock = false;
    }
    public void update() {
        if(timer == null) {
            timer = new ElapsedTime();
        }
        double dt = timer.seconds();
        timer.reset();
        fl.update(dt);
        bl.update(dt);
        fr.update(dt);
        br.update(dt);
    }
}