package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@TeleOp
public class BevelCentricMecanumTeleop extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL"); // 0 CHUB
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL"); // 2 CHUB
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");; //1 CHUB
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "bL"); //3 CHUB

        DcMotorEx flyWheel = hardwareMap.get(DcMotorEx.class, "fly");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "it");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
        Spindex spindex = new Spindex(hardwareMap);

         //1 CHUB
        //CRServoImplEx kicker = hardwareMap.get(CRServoImplEx.class, "kicker"); //1 CHUB
        //CRServoImplEx hood = hardwareMap.get(CRServoImplEx.class, "hood"); // 5 CHUB

        // Adjust motor directions for bevel drive layout
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // IMU setup
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Forward/backward is correct; flip strafe (x) and rotation (rx)
            double y = gamepad1.left_stick_y;     // correct already
            double x = -gamepad1.left_stick_x;    // flip strafe (left/right)
            double rx = -gamepad1.right_stick_x;  // flip rotation
            double flyPower = -0.85*gamepad2.left_stick_y; // flywheel control
            double turretPower = turret.getPower();

            if (gamepad1.options) imu.resetYaw();

            if (gamepad1.dpad_right) {
                turret.setPower(.4);
            } else if (gamepad1.dpad_left) {
                turret.setPower(-.4);
            } else {
                turret.setPower(0);
            }

//            if (gamepad2.x) {
//                spindex.setPowew(0.275);
//                sleep(343);
//            } else if (gamepad2.y) {
//                spindex.setPower(0.275);
//                sleep(167);
//            } else {
//                spindex.setPower(0);
//            }

//            if (gamepad1.dpad_up) {
//                hood.setPower(0.5);
//            } else if (gamepad1.dpad_down) {
//                hood.setPower(-0.5);
//            } else {
//                hood.setPower(0);
//            }

            if (gamepad1.right_trigger != 0) {
                intake.setPower(1);
            } else if (gamepad1.left_trigger != 0) {
                intake.setPower(-1);
            } else {
                intake.setPower(0.6);
            }

//            if (gamepad2.right_bumper) {
//                spindex.setPower(0.15);
//
//            } else if (gamepad2.left_bumper) {
//                spindex.setPower(-0.15);
//            } else {
//                spindex.setPower(0);
//            }

//            if (gamepad2.a) {
//                kicker.setPower(1);
//            } else if (gamepad2.b) {
//                kicker.setPower(-0.5);
//            } else {
//                kicker.setPower(0);
//            }






            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Apply field-centric transform
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // Strafing correction factor

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(-backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(-backRightPower);

            flyWheel.setPower(flyPower);

            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.addData("Strafe X", x);
            telemetry.addData("Rotation RX", rx);
            telemetry.addData("Turret thingies", turretPower);
            telemetry.update();
        }
    }
}
