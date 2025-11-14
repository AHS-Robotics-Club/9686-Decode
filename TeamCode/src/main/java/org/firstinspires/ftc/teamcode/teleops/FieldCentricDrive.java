package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemFC;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


@TeleOp(name = "Field Centric Drive FTCLib")

public class FieldCentricDrive extends CommandOpMode {


    private Motor fL, fR, bL, bR;


    private DriveCommand driveC;
    private DriveSubsystemFC driveS;
    private GamepadEx driverPad, gunnerPad;




    private Spindex spindex;

    private Kicker kicker;

    private Intake intake;
    private Turret turret;

    private Limelight3A limelight;

    @Override
    public void initialize() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);  // Set pipeline to AprilTag detection

        limelight.start();


        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        turret = new Turret(hardwareMap);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
//        spindex = new Motor(hardwareMap, "spindex");

        fL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        fL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);


        driverPad = new GamepadEx(gamepad1);
        gunnerPad = new GamepadEx(gamepad2);

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
                    spindex.stepForward();
                    intake.cycle();
                }

        );

        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
                    spindex.bigStepForward();
                    intake.cycle();
                }

        );

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() ->
                turret.spinLeft()


                );

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() ->
                turret.spinRight()


        );


        gunnerPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() ->
                kicker.kick()
        );

        gunnerPad.getGamepadButton(GamepadKeys.Button.A).whenReleased(() ->
                kicker.down()
        );


        driveS = new DriveSubsystemFC(fL, fR, bL, bR, imu, 1);
        driveC = new DriveCommand(driveS, driverPad::getLeftY, driverPad::getLeftX, driverPad::getRightX, 0.9);


        register(driveS);
        register(spindex);
        register(intake);
        register(turret);
        driveS.setDefaultCommand(driveC);


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.update();
        }


        telemetry.addData("Spindex Encoder", spindex.getCurrentPos());
        telemetry.addData("Target Spindex Position", spindex.getPidTarget());
        // telemetry.addData("Motor Power", spindex.getMotorPwr());
        telemetry.update();


    }


}
