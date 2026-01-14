package org.firstinspires.ftc.teamcode.tele.subsystems;

public class Pose {
    final public double x, y, theta;
    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    public Pose scalar(double k) {
        return new Pose(k * x, k * y, k * theta);
    }
    public Pose plus(Pose p) {
        return new Pose(this.x + p.x, this.y + p.y, this.theta + p.theta);
    }
    public Pose minus(Pose p) {
        return new Pose(this.x - p.x, this.y - p.y, this.theta - p.theta);
    }
    public double velocity() {
        return Math.hypot(this.x, this.y);
    }
    public Pose rotate(double angle) {
        return new Pose(
                this.x * Math.cos(angle) - this.y * Math.sin(angle),
                this.x * Math.sin(angle) + this.y * Math.cos(angle),
                this.theta + angle
        );
    }
}
