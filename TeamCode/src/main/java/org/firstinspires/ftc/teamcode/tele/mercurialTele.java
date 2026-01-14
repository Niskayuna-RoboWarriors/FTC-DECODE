package org.firstinspires.ftc.teamcode.tele;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.match;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitSeconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.tele.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.tele.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.tele.subsystems.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.tele.subsystems.Pose;

import dev.frozenmilk.dairy.mercurial.continuations.Actors;
import dev.frozenmilk.dairy.mercurial.continuations.Closure;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.continuations.Fiber;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

// Mercurial 2.0 uses a special runner at the moment
// its possible to recreate the way it works in other OpModes
// but the Mercurial program functions have some nice advantages

// Mercurial.teleop(ctx -> {})
// or Mercurial.autonomous(ctx -> {})
// will register an opmode when they are stored in a variable

// It is important to note that this does not use the OpMode or LinearOpMode classes

@SuppressWarnings("unused")
public class mercurialTele {
    private static class State {

    }
    public static final Mercurial.RegisterableProgram mercurialTele = Mercurial.teleop(ctx -> {
        State state = new State();
        IMU imu = ctx.hardwareMap().get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        //Localizer localizer = new OdometryLocalizer(ctx, new Pose(0, 0, 0), imu);
        MecanumDrive drive = new MecanumDrive(ctx);

        enum DriveState {
            driver,
            path
        }
        Actors.Actor<DriveState, DriveState> actor = Actors.actor(
                () -> DriveState.driver,
                (driveState, message) -> message,
                driveState -> match(driveState)
                        .branch(DriveState.driver,
                                exec(() -> {

                                }))
                        .branch(DriveState.path,
                                sequence(
                                        exec(() -> {

                                        }),
                                        exec(() -> driveState.set(DriveState.driver))
                                ))
                        .assertExhaustive()
        );
        ctx.schedule(actor);
        


        ctx.bindSpawn(
                // we can use the rising edge function to add a filter to the condition:
                // this only runs once when we press `gamepad1.a`, not every loop that it is pressed
                ctx.risingEdge(() -> ctx.gamepad1().a),
                // we haven't seen wait before, but it waits for the passed number in seconds
                sequence(
                        waitSeconds(1),
                        exec(() -> motor.setPower(1))
                )
        );

        // as long as gamepad1.a continues to return true,
        // the loop will continue to run
        // once it returns false,
        // the loop will be cancelled
        ctx.bindWhileTrue(
                () -> ctx.gamepad1().a,
                loop(exec(() -> motor.setPower(1)))
        );

        // we have some common utility functions we have seen in LinearOpMode
        // wait for start will run the scheduler until start is pressed
        ctx.waitForStart();

        ctx.telemetry().addLine("started!");

        // drop to scheduler will give up the rest of the op mode runtime to the scheduler
        ctx.dropToScheduler();

        // forgetting to call this will cause your op mode to end early
        // code after it will be run only after the opmode finishes

        // now, on to `Registers`
    });
}