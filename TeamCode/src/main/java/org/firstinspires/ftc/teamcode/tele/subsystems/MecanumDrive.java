package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import dev.frozenmilk.dairy.mercurial.ftc.Context;

public class MecanumDrive {
    Context ctx;
    DcMotorEx fl, fr, bl, br;
    Pid controllerfl, controllerfr, controllerbl, controllerbr;
    public MecanumDrive(Context ctx) {
        this.ctx = ctx;
        this.fl = ctx.hardwareMap().get(DcMotorEx.class, "frontLeft");
        this.fr = ctx.hardwareMap().get(DcMotorEx.class, "frontRight");
        this.bl = ctx.hardwareMap().get(DcMotorEx.class, "backLeft");
        this.br = ctx.hardwareMap().get(DcMotorEx.class, "backRight");
        this.fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
    }
    double pFrontLeftPower = 0;
    double pFrontRightPower = 0;
    double pBackLeftPower = 0;
    double pBackRightPower = 0;

    double motorEpsilon = 0.01;
    double joyStickZero = 0.01;
    double motorStopTryingThreshold = 0.05;

    boolean headingLock = true;
    boolean lockOnZeroAngularVelocity = false;
    double angularVelocityLockingThreshold = .3;
    double angularVelocityLockingThresholdDirection = 0;
    double headingLockDirection = 0;

    double headingPGain = 6.5 * 0.6;
    double headingDGain = 0.28;

    double loopRate = 0;

        while (opModeIsActive()){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        if (Math.abs(y) < joyStickZero) {
            y = 0;
        }
        if (Math.abs(x) < joyStickZero) {
            x = 0;
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        double yawAngularVelocity = angularVelocity.zRotationRate;
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);
        if (botHeading < 0) {
            botHeading = 2 * Math.PI + botHeading;
        }
        if (Math.abs(rx) < joyStickZero) {
            rx = 0;
            if (!headingLock) {
                headingLock = true;
                lockOnZeroAngularVelocity = true;
                angularVelocityLockingThresholdDirection = -Math.signum(yawAngularVelocity);
                headingLockDirection = botHeading;
            }
            if (lockOnZeroAngularVelocity) {
                if (Math.abs(yawAngularVelocity) < angularVelocityLockingThreshold
                        || Math.signum(yawAngularVelocity) == angularVelocityLockingThresholdDirection) {
                    headingLockDirection = botHeading;
                    lockOnZeroAngularVelocity = false;
                }
            }
        } else {
            headingLock = false;
        }

        if (gamepad1.leftBumperWasPressed()) {
            headingLockDirection = 0;
        }

        if (headingLock) {
            double headingDifference = botHeading - headingLockDirection;
            if (headingDifference > Math.PI) {
                headingDifference = headingDifference - 2 * Math.PI;
            }
            if (headingDifference < -Math.PI) {
                headingDifference = 2 * Math.PI - headingDifference;
            }
            rx = headingPGain * headingDifference + headingDGain * yawAngularVelocity;
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (Math.abs(pFrontLeftPower-frontLeftPower) > motorEpsilon) {
            if (Math.abs(frontLeftPower) < motorStopTryingThreshold) frontLeftPower = 0;
            frontLeft.setPower(frontLeftPower);
            pFrontLeftPower = frontLeftPower;
        }
        if (Math.abs(pBackLeftPower-backLeftPower) > motorEpsilon) {
            if (Math.abs(backLeftPower) < motorStopTryingThreshold) backLeftPower = 0;
            backLeft.setPower(backLeftPower);
            pBackLeftPower = backLeftPower;
        }
        if (Math.abs(pFrontRightPower-frontRightPower) > motorEpsilon) {
            if (Math.abs(frontRightPower) < motorStopTryingThreshold) frontRightPower = 0;
            frontRight.setPower(frontRightPower);
            pFrontRightPower = frontRightPower;
        }
        if (Math.abs(pBackRightPower-backRightPower) > motorEpsilon) {
            if (Math.abs(backRightPower) < motorStopTryingThreshold) backRightPower = 0;
            backRight.setPower(backRightPower);
            pBackRightPower = backRightPower;
        }

        telemetry.addData("Heading", botHeading);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Rad/Sec", angularVelocity.zRotationRate);
        loopRate = loopRate * 0.95 + 0.05/timer.time();
        telemetry.addData("Loop Rate", "%.2f Hz", loopRate);
        telemetry.update();
}
