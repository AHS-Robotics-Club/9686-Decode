package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemFC;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;


@TeleOp(name = "Field Centric Drive FTCLib")

public class FieldCentricDrive extends CommandOpMode {
    private PIDController testPID = new PIDController(0.8, 0.1, 0.1);

    private Motor fL, fR, bL, bR;



    private DriveCommand driveC;
    private DriveSubsystemFC driveS;
    private GamepadEx driverPad, gunnerPad;

    private int targetPosition = 1365;



    private Spindex spindex;

    @Override
    public void initialize() {



        spindex = new Spindex(hardwareMap);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
//        spindex = new Motor(hardwareMap, "spindex");

        fL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        bL.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        fR.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.motor.setDirection(DcMotorSimple.Direction.REVERSE);




        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);


        driverPad = new GamepadEx(gamepad1);
        gunnerPad = new GamepadEx(gamepad2);

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() ->
                spindex.nextPos()
        );





        driveS = new DriveSubsystemFC(fL, fR, bL, bR, imu, 0.9);
        driveC = new DriveCommand(driveS, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX, 0.9);



        register(driveS);
        register(spindex);
        driveS.setDefaultCommand(driveC);




        telemetry.addData("Spindex Encoder", spindex.getCurrentPos());
       // telemetry.addData("Motor Power", spindex.getMotorPwr());
        telemetry.update();







    }


    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }
}
