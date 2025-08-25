package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotorEx
import dev.frozenmilk.dairy.core.dependency.Dependency
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.subsystems.Subsystem
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.dairy.core.Feature
import dev.frozenmilk.dairy.core.FeatureRegistrar
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation
import java.lang.annotation.Inherited

/*
a little test for the diary mercurial subsystem
this takes motor of motor named "name"
 */
object SubsystemTest : Subsystem {
    @Target(AnnotationTarget.CLASS)
    @Retention(AnnotationRetention.RUNTIME)
    @MustBeDocumented
    @Inherited
    annotation class Attach
    override var dependency: Dependency<*> = Subsystem.DEFAULT_DEPENDENCY and
            SingleAnnotation(Attach::class.java)
    /*
    override var dependency: Dependency<*> = Dependency {
        opMode: Wrapper, resolvedFeatures: List<Feature>, yielding: Boolean ->
    }*/

    override fun preUserInitHook(opMode: Wrapper) {}

    override fun postUserInitHook(opMode: Wrapper) {}

    override fun preUserInitLoopHook(opMode: Wrapper) {}

    override fun postUserInitLoopHook(opMode: Wrapper) {}

    override fun preUserStartHook(opMode: Wrapper) {}

    override fun postUserStartHook(opMode: Wrapper) {}

    override fun preUserLoopHook(opMode: Wrapper) {}

    override fun postUserLoopHook(opMode: Wrapper) {}

    override fun preUserStopHook(opMode: Wrapper) {}

    override fun postUserStopHook(opMode: Wrapper) {}

    private val motor by subsystemCell {
        FeatureRegistrar.activeOpMode.hardwareMap.get(DcMotorEx::class.java, "name")
    }

    fun simpleCommand(): Lambda {
        return Lambda("simple")
            .addRequirements(this)
            .setInit { this.motor.power = 0.4 }
            .setEnd { interrupted: Boolean? ->
                if (!interrupted!!) this.motor.power = 0.0
            }
    }

    // cleanup differs from postUserStopHook, it runs after the OpMode has completely stopped,
    // and is guaranteed to run, even if the OpMode stopped from a crash.
    override fun cleanup(opMode: Wrapper) {}
}
