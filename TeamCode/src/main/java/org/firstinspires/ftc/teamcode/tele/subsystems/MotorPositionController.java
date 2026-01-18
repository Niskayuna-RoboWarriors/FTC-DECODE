package org.firstinspires.ftc.teamcode.tele.subsystems;

public class MotorPositionController {
    MotorVelocityController controller;

    Pid pid;

    public MotorPositionController(MotorVelocityController controller, double kp, double kd, double ki) {
        this.controller = controller;
        this.pid = new Pid(kp, kd, ki);
    }
    public void setTarget(double target) {
        pid.setTarget(target);
    }
    public void update(double dt) {
        double power = pid.pid(controller.motor.getCurrentPosition(), dt);
        controller.setTarget(power);
        controller.update(dt);
    }
}
