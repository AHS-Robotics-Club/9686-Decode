package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx kicker = hardwareMap.get(ServoImplEx.class, "kicker");
        kicker.setPwmRange(new PwmControl.PwmRange(500, 2500));

        double pos = 0.5;
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) pos += 0.001;
            if (gamepad1.y) pos -= 0.001;
            pos = Math.max(0.0, Math.min(1.0, pos));
            kicker.setPosition(pos);

            telemetry.addData("Servo Pos", pos);
            telemetry.update();
        }
    }
}
