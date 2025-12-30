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
        DcMotorEx forwardEncoder = hardwareMap.get(DcMotorEx.class, "forwardEncoder");
        DcMotorEx strafeEncoder = hardwareMap.get(DcMotorEx.class, "strafeEncoder");

        forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("position: ", forwardEncoder.getCurrentPosition());
            telemetry.addData("position: ", strafeEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
