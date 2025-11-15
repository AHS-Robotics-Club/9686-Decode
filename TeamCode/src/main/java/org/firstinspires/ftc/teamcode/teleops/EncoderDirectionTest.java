package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Direction Test (Fixed)")
public class EncoderDirectionTest extends LinearOpMode {

    // Use this constant (1 or -1) to flip the encoder sign in one place.
    // We'll start with -1, since you were manually negating it.
    private static final int ENCODER_SIGN = -1;

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        DcMotor spindex = hardwareMap.dcMotor.get("spindex");

        // 1. Set the motor direction. This controls the physical movement relative to power.
        // If power 0.15 makes the spindex move in the desired "forward" direction, use FORWARD.
        // If it moves backward, use REVERSE.
        spindex.setDirection(DcMotorSimple.Direction.REVERSE);

        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Using RUN_USING_ENCODER is generally smoother than RUN_WITHOUT_ENCODER
        // when the motor has an encoder connected, even for simple power control.
        spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // 2. Read the raw ticks and apply the sign correction (ENCODER_SIGN).
            // This is where your previous negation was. We make it a constant for clarity.
            int currentPosition = spindex.getCurrentPosition() * ENCODER_SIGN;

            // --- Motor Control ---
            if (gamepad1.a) {
                spindex.setPower(0.15); // Positive power
            } else if (gamepad1.b) {
                spindex.setPower(-0.15); // Negative power
            } else {
                spindex.setPower(0);
            }

            // --- Telemetry for Debugging ---
            telemetry.addData("1. Raw Ticks", spindex.getCurrentPosition());
            telemetry.addData("2. Direction Setting", spindex.getDirection());
            // FIX: Concatenate the string manually instead of using C-style format specifiers.
            telemetry.addData("3. Calculated Position", "Sign: " + ENCODER_SIGN + " | Ticks: " + currentPosition);

            String movement = "Stopped";
            if(gamepad1.a) movement = "A (Positive Power)";
            if(gamepad1.b) movement = "B (Negative Power)";
            telemetry.addData("4. Command", movement);

            telemetry.update();
        }
    }
}