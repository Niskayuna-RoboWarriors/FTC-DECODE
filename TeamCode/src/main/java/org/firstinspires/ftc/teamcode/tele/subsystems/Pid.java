package org.firstinspires.ftc.teamcode.tele.subsystems;

public class Pid {
    double Kp, Kd, Ki;
    double previousError = 0;
    double integral;
    double target;
    public Pid(double Kp, double Kd, double Ki) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
    }

    public void setTarget(double target) {
        this.target = target;
    }
    public double pid(double currentPosition, double dt) {
        double error = this.target - currentPosition;
        double de_dt = (error - this.previousError) / dt;
        integral += error * dt;
        this.previousError = error;
        return Kp * error + Kd * de_dt + Ki * this.integral;
    }
}

// Kp, Kd, Ki
//