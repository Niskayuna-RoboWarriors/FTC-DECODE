package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import dev.frozenmilk.dairy.mercurial.ftc.Context;

public abstract class Localizer {
    public abstract Pose getPosition();
    public abstract void update();
}
