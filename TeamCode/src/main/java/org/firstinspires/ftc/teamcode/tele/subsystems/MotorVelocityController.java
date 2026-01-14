package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class MotorVelocityController {
    DcMotorEx motor;
    Pid pid;
    double prevPower = 0;
    public MotorVelocityController(DcMotorEx motor, Pid pid) {
        this.motor = motor;
        this.pid = pid;
    }

    public MotorVelocityController(DcMotorEx motor, double kp, double kd) {

    }

    public void setTarget(double target) {
        pid.setTarget(Range.scale(target, -1, 1, -6000, 6000));
    }

    public void update(double dt) {
        double power = pid.pid(motor.getVelocity(), dt);
        if (Math.abs(prevPower-power) > 0.01) {
            if (Math.abs(power) < 0.01) power = 0;
            motor.setPower(power);
            prevPower = power;
        }
    }
}
