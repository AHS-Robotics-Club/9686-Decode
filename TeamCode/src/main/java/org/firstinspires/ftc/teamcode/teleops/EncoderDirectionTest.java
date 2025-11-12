package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// This may vary depending on what implementation you are using.


@TeleOp
public class EncoderDirectionTest extends LinearOpMode {



    @Override
public void runOpMode() {
    // Put all of your initialization here.
    DcMotor spindex = hardwareMap.dcMotor.get("spindex");
    spindex.setDirection(DcMotorSimple.Direction.REVERSE);
    spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    spindex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    waitForStart();

    int targetPosition = -spindex.getCurrentPosition();

    // We will use this variable to determine if we want the PIDF to run.
    boolean lastA = false;



    while (opModeIsActive()) {

        int position = -spindex.getCurrentPosition();

        if (gamepad1.a) {
            spindex.setPower(0.15);


        } else if (gamepad1.b) {
            spindex.setPower(-0.15);
        } else spindex.setPower(0);



        // This is a rising-edge detector that runs if and only if "a" was pressed this loop.







            // Sets the slide motor power according to the PIDF output.

        telemetry.addData("Spindex Encoder", position);


        telemetry.update();


        }
    }
}


