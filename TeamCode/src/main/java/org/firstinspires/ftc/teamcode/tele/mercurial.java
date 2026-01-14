package org.firstinspires.ftc.teamcode.tele;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitSeconds;

import com.qualcomm.robotcore.hardware.DcMotorEx;

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
public class mercurial {
    // this will be registered as TeleOp with the name 'myFirstMercurialOpMode'
    // in order for a mercurial program to be registered, it must be both public and static
    // so you can make it private to disable it
    public static final Mercurial.RegisterableProgram myFirstMercurialOpMode = Mercurial.teleop(ctx -> {
        // provided values:

        // the scheduler:
        // allows us to run Continuations
        // however, you probably won't need to interact with it directly
        ctx.scheduler();

        // gamepads:
        // standard from the sdk
        ctx.gamepad1();
        ctx.gamepad2();

        // hardwareMap:
        // standard from the sdk
        ctx.hardwareMap();

        // telemetry:
        // standard from the sdk
        ctx.telemetry();

        // current state:
        // INIT, LOOP, STOP
        ctx.state();

        // helpers:
        ctx.inInit(); // true if state == INIT
        ctx.inLoop(); // true if state == LOOP
        ctx.isActive(); // true if either of the above are true

        // we can access the HardwareMap immediately
        DcMotorEx motor = ctx.hardwareMap().get(DcMotorEx.class, "");

        // allows us to schedule a Continuation
        ctx.schedule(exec(() -> {}));
        ctx.schedule(
                sequence(
                        exec(() -> {}),
                        exec(() -> {}),
                        exec(() -> {}),
                        exec(() -> {})
                )
        );
        // this one will run forever
        Fiber fiber = ctx.schedule(loop(exec(() -> {})));
        // so we can grab the fiber from it
        // and cancel it:
        Fiber.CANCEL(fiber);
        // so that it doesn't run forever

        // in addition to `schedule`
        // we have some helpers to set up loops that poll for events,
        // and if they become true, they run a Continuation for us

        // these are generally better to use than schedule,
        // unless you're scheduling an infinite loop,
        // or scheduling a one off to run immediately

        // every single loop that `gamepad1.a` returns true
        // this will start an infinite loop that sets the motor power to 1
        // however, if the infinite loop is already running
        // then it will cancel the infinite loop, and replace it with a new copy
        ctx.bindExec(
                // condition
                () -> ctx.gamepad1().a,
                // run
                loop(exec(() -> motor.setPower(1)))
        );

        // bindSpawn is like bindExec, but will not cancel already running Fibers
        // every time `gamepad1.a` is pressed, this will wait 1 second, then turn on the motor
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