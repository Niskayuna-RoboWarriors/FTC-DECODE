package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.mercurial.continuations.Closure;
import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.ftc.Context;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.parallel;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import java.util.concurrent.atomic.AtomicBoolean;

public class Turret {
    Context ctx;
    MotorPositionController positionalControl;
    MotorVelocityController velocityControl;
    static double minPos;
    static double maxPos;
    ElapsedTime timer;
    boolean localizing = false;

    public Turret(Context ctx) {
        this.ctx = ctx;
        DcMotorEx motor = ctx.hardwareMap().get(DcMotorEx.class, "turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        velocityControl = new MotorVelocityController(ctx, motor, 1, 0);
        positionalControl = new MotorPositionController(velocityControl, 0.01, 0, 0);
    }

    public Closure autoFindMaxRange() {
        AtomicBoolean flag = new AtomicBoolean(false);
        return sequence(
                exec(() -> this.localizing = true),
                exec(() -> {
                    flag.set(false);
                    velocityControl.setCurrentLimit(1, 0);
                    velocityControl.ifOverCurrentCall(() -> {
                        minPos = velocityControl.motor.getCurrentPosition();
                        flag.set(true);
                        velocityControl.setTarget(0);
                    });
                    velocityControl.setTarget(-0.1);
                    waitUntil(flag::get);
                }),
                exec(() -> {
                    flag.set(false);
                    velocityControl.ifOverCurrentCall(() -> {
                        maxPos = velocityControl.motor.getCurrentPosition();
                        flag.set(true);
                        velocityControl.setTarget(0);
                    });
                    velocityControl.setTarget(0.1);
                    waitUntil(flag::get);
                }),
                exec(() -> {
                    velocityControl.setCurrentLimit(8, 5);
                    velocityControl.ifOverCurrentCall(null);
                }),
                exec(() -> this.localizing = false),
                exec(() -> positionalControl.setTarget((minPos+maxPos)/2))
        );
    }

    public double position() {
        return velocityControl.motor.getCurrentPosition();
    }

    public void setTarget(double target) {
        positionalControl.setTarget(Range.clip(target,minPos,maxPos));
    }

    public void update() {
        if(timer == null) {
            timer = new ElapsedTime();
        }
        double dt = timer.seconds();
        timer.reset();
        if (localizing) velocityControl.update(dt);
        else positionalControl.update(dt);
    }
}
