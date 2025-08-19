package org.firstinspires.ftc.teamcode.tele

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundBooleanSupplier
import dev.frozenmilk.mercurial.bindings.BoundDoubleSupplier
import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Advancing
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.StateMachine
import dev.frozenmilk.mercurial.commands.util.Wait
import dev.frozenmilk.util.cell.RefCell

@Mercurial.Attach
//@KotlinSubsystem.Attach
@TeleOp(name = "Competition")
class Competition : OpMode() {
    override fun init() {
        val driver = BoundGamepad(SDKGamepad(gamepad1))
        // bindings to driver gamepad
        driver.a
            .onTrue(Lambda("hi"))
            .onFalse(Lambda("bye"))

        val operator = BoundGamepad(SDKGamepad(gamepad2))
        //bindings to operator gamepad
        operator.a
            .whileTrue(Lambda("extend linear slide"))
            .onFalse(Lambda("hold linear slides"))
    }

    override fun loop() {
        // unnecessary with Mercurial
    }
}