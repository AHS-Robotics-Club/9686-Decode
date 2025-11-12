package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BevelCentricMecanumDriveOnly_RobotCentric", group = "TeleOp")
public class BevelCentricMecanumDriveOnly_RobotCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drive motors ---
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL"); // 0 CHUB
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");  // 2 CHUB
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR"); // 1 CHUB
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");  // 3 CHUB

        // --- Motor directions ---
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready to start (Robot-Centric)");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- Read joystick inputs ---
            double y = -gamepad1.left_stick_y;   // Forward is -Y
            double x = gamepad1.left_stick_x;    // Strafe
            double rx = gamepad1.right_stick_x;  // Rotation

            // --- Compute motor powers (standard mecanum math) ---
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // --- Apply powers ---
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- Telemetry ---
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
