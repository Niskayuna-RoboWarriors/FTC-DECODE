package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        pid.setTarget(target);
    }

    public void update() {
        if (Math.abs(prevPower-pid.pid(motor.getVelocity(), dt)) > ) {
            if (Math.abs(frontLeftPower) < motorStopTryingThreshold) frontLeftPower = 0;
            frontLeft.setPower(frontLeftPower);
            pFrontLeftPower = frontLeftPower;
        }
    }
}
