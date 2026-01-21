package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@Config
@TeleOp(name = "It gets to a point")

public class ShooterVelocityTest extends CommandOpMode {


    private DcMotorEx flywheel;

    private Kicker kicker;

    private Spindex spindex;
    private FtcDashboard dash;

    private GamepadEx gunnerPad;
    public static double kP = 0.000455; // Default PID constants
    public static double kI = 0.0000000;
    public static double kD = 0.00000;

    public static double kF = 0;

    public static double currentVeloTicks = 0;

    public static double targetVeloTicks = 0;

    private double output = 0;

    private double error = 0;

    private PIDFController flyPIDF = new PIDFController(0.00455, 0, 0, 0);


    @Override
    public void initialize() {

        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        spindex = new Spindex(hardwareMap);

        kicker = new Kicker(hardwareMap);

        gunnerPad = new GamepadEx(gamepad2);

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dash = FtcDashboard.getInstance();

        register(spindex);
        register(kicker);

        gunnerPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            kicker.down();
            spindex.bigStepForward();

        });
        gunnerPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            kicker.down();
            spindex.stepForward();

        });

    }




    @Override
    public void run() {


//        double flypwr = gamepad2.left_stick_y;
//
//
//        flywheel.setPower(flypwr);


        currentVeloTicks = flywheel.getVelocity();

        flyPIDF.setPIDF(kP, kI, kD, kF);


        output = flyPIDF.calculate(currentVeloTicks, targetVeloTicks);

        flywheel.setPower(output);


        error = Math.abs(targetVeloTicks) - Math.abs(currentVeloTicks);




        if (gamepad2.a) kicker.kick();
        else kicker.down();








        telemetry.addData("Velocity in ticks", flywheel.getVelocity());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("PID kP", kP);
        packet.put("PID kI", kI);
        packet.put("PID kD", kD);
        packet.put("Current Velo", currentVeloTicks);
        packet.put("Target Velo", targetVeloTicks);
        packet.put("Error", error);
        dash.sendTelemetryPacket(packet);


    }


}
