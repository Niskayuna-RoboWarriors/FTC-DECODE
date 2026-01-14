package org.firstinspires.ftc.teamcode.tele.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="intake", group="testing")
public class intake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        waitForStart();
        while(opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_y);
        }
    }
}
