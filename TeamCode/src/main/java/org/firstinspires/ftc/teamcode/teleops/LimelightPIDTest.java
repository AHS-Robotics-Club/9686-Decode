package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "It gets to a point bro")

public class LimelightPIDTest extends CommandOpMode {


    public Limelight limelight;
    public Turret turret;
    private FtcDashboard dash;

    public static double currentTx = 0;

    private GamepadEx gunnerPad;
    public static double kP = 0.000; // Default PID constants
//    public static double kI = 0.0000000;
//    public static double kD = 0.00000;

//    public static double kF = 0;

    private double output = 0;

    private double error = 0;

    private PController limelightP = new PController(0.00455);


    @Override
    public void initialize() {

        gunnerPad = new GamepadEx(gamepad2);

        limelight = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap);

        dash = FtcDashboard.getInstance();

        register(limelight);
        register(turret);








    }




    @Override
    public void run() {
        LLResult result = limelight.getRawResult();

        boolean hasTarget = limelight.hasTarget();

        if (hasTarget && result != null) {
            currentTx = result.getTx();
            output = limelightP.calculate(currentTx, 0);
            turret.manual(output);
        } else {
            output = 0;
            turret.manual(0); // Stop turret if no target
        }

        limelightP.setP(kP);

        if (gamepad1.y) limelight.switchPipelineRed();
        if (gamepad1.b) limelight.switchPipelineBlue();

        // Dashboard telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("PID kP", kP);
        packet.put("currentTx", currentTx);

        if (hasTarget && result != null) {
            packet.put("tx", result.getTx());
            packet.put("ty", result.getTy());
            packet.put("ta", result.getTa());
        } else {
            packet.put("tx", "No Target");
            packet.put("ty", "No Target");
            packet.put("ta", "No Target");
        }

        dash.sendTelemetryPacket(packet);
        telemetry.update();
    }

}
