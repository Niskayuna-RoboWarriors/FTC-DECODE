package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="readEncoder", group="testing")
public class readEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "intake");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("position: ", encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
