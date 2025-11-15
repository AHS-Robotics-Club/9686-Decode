package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Spindex F Finder Manual", group = "Test")
public class SpindexFFinderManual extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx spindexMtr = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexMtr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexMtr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Manual F Finder");
        telemetry.addLine("DPAD_UP = increase power, DPAD_DOWN = decrease power");
        telemetry.addLine("Watch encoder. First movement = F value.");
        telemetry.update();

        waitForStart();

        double power = 0.0;
        int lastPos = spindexMtr.getCurrentPosition();
        boolean hasMoved = false;

        while (opModeIsActive()) {
            // Manual adjustment
            if (gamepad1.dpad_up) {
                power += 0.01;
            }
            if (gamepad1.dpad_down) {
                power -= 0.01;
            }

            // Clamp power
            power = Math.max(0.0, Math.min(1.0, power));

            // Set motor power
            spindexMtr.setPower(power);

            int currentPos = spindexMtr.getCurrentPosition();

            // Detect first movement
            if (!hasMoved && Math.abs(currentPos - lastPos) > 5) {
                hasMoved = true;
                telemetry.addLine("Motor started moving!");
                telemetry.addData("Approx F value", power);
            }

            lastPos = currentPos;

            telemetry.addData("Current Power", power);
            telemetry.addData("Encoder", currentPos);
            telemetry.addData("Has Moved?", hasMoved);
            telemetry.update();

            sleep(50); // small delay for motor response
        }

        // Stop motor when OpMode ends
        spindexMtr.setPower(0);
    }
}
