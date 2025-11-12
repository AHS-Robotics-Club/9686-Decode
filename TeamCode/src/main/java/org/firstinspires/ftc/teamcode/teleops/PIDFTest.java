package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // This example is for a LinearOpMode, though a similar idea applies to regular OpModes.
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// This may vary depending on what implementation you are using.
import org.firstinspires.ftc.teamcode.controllers.PIDFController; // This may vary depending on what implementation you are using.

@TeleOp
public class PIDFTest extends LinearOpMode {

    // This line creates a PIDF controller named examplePIDF that has coefficients of:
    // kP = 0
    // kI = 0
    // kD = 0
    // kF = 0
    //private PIDFController examplePIDF = new PIDFController(0, 0, 0, 0);

    // This line creates a PID controller named examplePID that has coefficients of:
    // kP = 0
    // kI = 0
    // kD = 0
    private PIDController testPID = new PIDController(0.0003, 0.0000, 0.0);

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

        if (gamepad1.a && !lastA) {
            targetPosition = position + (8192/6);
//            if (position >=0)
//                targetPosition = position + 8192/6;
//            else
//                targetPosition = position - 8192/6;

        }

        lastA = gamepad1.a;

        // This is a rising-edge detector that runs if and only if "a" was pressed this loop.







            // Sets the slide motor power according to the PIDF output.
            spindex.setPower(testPID.calculate(position, targetPosition));
            telemetry.addData("Spindex Encoder", position);
            telemetry.addData("TargetPosn", targetPosition);
        telemetry.addData("PID Power", testPID.calculate(spindex.getCurrentPosition(), targetPosition));
        telemetry.update();


        }
    }
}


