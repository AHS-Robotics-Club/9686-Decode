package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Forward 1 Second", group = "Auto")
public class ForwardAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // === Motor mapping (same as your TeleOp) ===
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("fL"); // 0 CHUB
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("bL"); // 2 CHUB
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fR"); // 1 CHUB
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("bR"); // 3 CHUB

        // === Match your TeleOp directions ===
        frontLeftMotor.setDirection((DcMotorSimple.Direction.FORWARD));
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: reset encoders for safety
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready to drive forward for 1 second");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Drive forward ===
        double power = 0.4  ; // Adjust speed as needed
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);

        sleep(2000); // 1 second

        // === Stop motors ===
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addLine("Done!");
        telemetry.update();
    }
}
