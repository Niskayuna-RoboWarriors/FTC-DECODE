package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.frozenmilk.dairy.mercurial.ftc.Context;

public class OdometryLocalizer extends Localizer {
    DcMotorEx front, strafe;
    IMU imu;
    Pose position;
    Context ctx;
    public OdometryLocalizer(Context ctx, Pose starting, IMU imu) {
        this.ctx = ctx;
        this.position = starting;
        this.front = ctx.hardwareMap().get(DcMotorEx.class, "front");
        this.strafe = ctx.hardwareMap().get(DcMotorEx.class, "strafe");
        this.imu = imu;
    }

    @Override
    public Pose getPosition() {
        return this.position;
    }

    @Override
    public void update() {
        position = new Pose(
                position.x,
                position.y,
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );
    }
}