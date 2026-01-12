//package org.firstinspires.ftc.teamcode.teleops;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.controller.PController;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.subsystems.Kicker;
//import org.firstinspires.ftc.teamcode.subsystems.Limelight;
//import org.firstinspires.ftc.teamcode.subsystems.Spindex;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//
//@Config
//@TeleOp(name = "It gets to a point bro")
//
//public class LimelightPIDTest extends CommandOpMode {
//
//
//    public Limelight limelight;
//    public Turret turret;
//    private FtcDashboard dash;
//
//    public static double currentTx = 0;
//
//    private GamepadEx gunnerPad;
//    public static double kP = 0.000; // Default PID constants
////    public static double kI = 0.0000000;
////    public static double kD = 0.00000;
//
////    public static double kF = 0;
//
//    private double output = 0;
//
//    private double error = 0;
//
//    private PController limelightP = new PController(0.00455);
//
//
//    @Override
//    public void initialize() {
//
//        gunnerPad = new GamepadEx(gamepad2);
//
//        limelight = new Limelight(hardwareMap);
//        turret = new Turret(hardwareMap);
//
//        dash = FtcDashboard.getInstance();
//
//        register(limelight);
//        register(turret);
//
//
//    }
//
//
//    @Override
//    public void run() {
//        LLResult result = limelight.getRawResult();
//
//        boolean hasTarget = limelight.hasTarget();
//
//        if (hasTarget && result != null) {
//            currentTx = result.getTx();
//            output = limelightP.calculate(currentTx, 0);
//            turret.manual(output);
//        } else {
//            output = 0;
//            turret.manual(0); // Stop turret if no target
//        }
//
//        limelightP.setP(kP);
//
//        if (gamepad1.y) limelight.switchPipelineRed();
//        if (gamepad1.b) limelight.switchPipelineBlue();
//
//        // Dashboard telemetry
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("PID kP", kP);
//        packet.put("currentTx", currentTx);
//
//        if (hasTarget && result != null) {
//            packet.put("tx", result.getTx());
//            packet.put("ty", result.getTy());
//            packet.put("ta", result.getTa());
//        } else {
//            packet.put("tx", "No Target");
//            packet.put("ty", "No Target");
//            packet.put("ta", "No Target");
//        }
//
//        dash.sendTelemetryPacket(packet);
//        telemetry.update();
//    }
//
//}


package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "Limelight PID Tuning")
public class LimelightPIDTest extends CommandOpMode {

    /* ================= Dashboard Tunables ================= */

    public static double kP = 0.015;
    public static double kI = 0.0;
    public static double kD = 0.001;

    public static double DEADBAND = 0.5;     // degrees of tx
    public static double MIN_POWER = 0.05;   // static friction kick
    public static double MAX_POWER = 0.6;    // safety clamp

    public static double TARGET_TX = 0.0;    // usually zero

    /* ================= Hardware ================= */

    private Turret turret;
    private Limelight limelight;

    /* ================= PID ================= */

    private PIDController pid;

    private FtcDashboard dashboard;

    @Override
    public void initialize() {

        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(DEADBAND);

        dashboard = FtcDashboard.getInstance();

        register(turret);
        register(limelight);
    }

    @Override
    public void run() {
        super.run();

        // Update PID constants live
        pid.setPID(kP, kI, kD);

        TelemetryPacket packet = new TelemetryPacket();

        boolean hasTarget = limelight.hasTarget();

        packet.put("Has Target", hasTarget);

        if (!hasTarget) {
            turret.stop();
            packet.put("Turret Power", 0);
            dashboard.sendTelemetryPacket(packet);
            return;
        }

        double tx = limelight.getTx();
        double error = TARGET_TX - tx;

        // Deadband check
        if (Math.abs(error) <= DEADBAND) {
            turret.stop();
            packet.put("Within Deadband", true);
            packet.put("Turret Power", 0);
            dashboard.sendTelemetryPacket(packet);
            return;
        }

        // PID calculation
        double output = pid.calculate(tx, TARGET_TX);

        // Static friction feedforward
        if (Math.abs(output) > 0.01) {
            output += Math.signum(output) * MIN_POWER;
        }

        // Clamp
        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        // Apply power
        turret.manual(output);

        /* ================= Telemetry ================= */

        packet.put("tx", tx);
        packet.put("Target tx", TARGET_TX);
        packet.put("Error", error);
        packet.put("PID Output", output);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);

        dashboard.sendTelemetryPacket(packet);
    }
}
