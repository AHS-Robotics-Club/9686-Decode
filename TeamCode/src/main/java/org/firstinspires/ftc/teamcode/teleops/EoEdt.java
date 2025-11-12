package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Evening of Excellence", group = "TeleOp")
public class EoEdt extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Drive motors ---
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL"); // 0 CHUB
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");  // 2 CHUB
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR"); // 1 CHUB
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");  // 3 CHUB (fixed from your original)

        // --- Motor directions ---
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- IMU setup ---
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- Read joystick inputs ---
            double y = -gamepad1.left_stick_y;   // Forward is -Y
            double x = gamepad1.left_stick_x;    // Strafe
            double rx = gamepad1.right_stick_x;  // Rotation

            // --- Reset heading if needed ---
            if (gamepad1.options) imu.resetYaw();

            // --- Field-centric correction ---
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // Small strafe correction

            // --- Calculate motor powers ---
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // --- Apply powers ---
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- Telemetry ---
            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
