package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.mercurial.ftc.Context;

public class MotorVelocityController {
    Context ctx;
    DcMotorEx motor;
    Pid pid;
    double prevPower = 0;
    Runnable overcurrentCall;
    double currentLimit = 8;
    double overcurrentIntegrator = 0;
    double overcurrentIntegratorThreshold = 5;
    boolean overcurrentTriggered = false;
    ElapsedTime overcurrentTimer;

    public MotorVelocityController(Context ctx, DcMotorEx motor, Pid pid) {
        this.ctx = ctx;
        this.motor = motor;
        this.pid = pid;
    }

    public MotorVelocityController(Context ctx, DcMotorEx motor, double kp, double kd) {
        this.ctx = ctx;
        this.motor = motor;
        this.pid = new Pid(kp, kd, 0);
    }

    public void setTarget(double target) {
        pid.setTarget(Range.scale(target, -1, 1, -6000, 6000));
    }

    public void setCurrentLimit(double limit, double integratorThreshold) {
        this.currentLimit = limit;
        motor.setCurrentAlert(currentLimit, CurrentUnit.AMPS);
    }

    public void ifOverCurrentCall(Runnable callback) {
        this.overcurrentCall = callback;
    }

    public void update(double dt) {
        double power = pid.pid(motor.getVelocity(), dt);
        power = Range.clip(power, -1, 1);
        if (Math.abs(prevPower-power) > 0.01) {
            if (Math.abs(power) < 0.01) power = 0;
            if (motor.isOverCurrent() || overcurrentIntegrator > 0) {
                double current = motor.getCurrent(CurrentUnit.AMPS);
                overcurrentIntegrator += (current-currentLimit) * dt;
                if (overcurrentIntegrator < 0) {
                    overcurrentIntegrator = 0;
                }
                if (overcurrentIntegrator > overcurrentIntegratorThreshold) {
                    if (overcurrentIntegrator > 100) throw new Error("Oop! Motor overcurrent.");
                    if (overcurrentCall == null) {
                        overcurrentTriggered = true;
                        if (overcurrentTimer == null) {
                            overcurrentTimer = new ElapsedTime();
                        } else {
                            overcurrentTimer.reset();
                        }
                    } else {
                        overcurrentCall.run();
                    }
                } else if (current > currentLimit) {
                    double currentScaleFactor = currentLimit / current;
                    if (Math.signum(power) == Math.signum(prevPower)) {
                        power = Math.signum(power) * currentScaleFactor * Math.min(Math.abs(prevPower),Math.abs(power));
                    }
                }
            }
            if (overcurrentTimer != null && overcurrentTimer.seconds() < 5) {
                ctx.telemetry().addData("!!! Overcurrent detected: ", motor.getDeviceName());
                ctx.telemetry().addData("> Halving motor output for 5s: ", overcurrentTimer.seconds());
                power /= 2;
            }
            motor.setPower(power);
            prevPower = power;
        }
    }
}
