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

    private double currentTx = 0;

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

        dash = FtcDashboard.getInstance();

        register(limelight);
        register(turret);








    }




    @Override
    public void run() {
        LLResult result = limelight.getRawResult();


        if (result != null) {
            currentTx = result.getTx();
        }


        output = limelightP.calculate(currentTx, 0);

        turret.manual(output);

        limelightP.setP(kP);

        if (gamepad1.y) {



            limelight.switchPipelineRed();

        }

        if (gamepad1.b) {
            limelight.switchPipelineBlue();
        }



        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("zero", 0);
        packet.put("PID kP", kP);
//        packet.put("PID kI", kI);
//        packet.put("PID kD", kD);

        if (result != null) {
            packet.put("ta", result.getTa());
            packet.put("ty", result.getTy());
            packet.put("tx", result.getTx());
            packet.put("Error", error);
        }
        dash.sendTelemetryPacket(packet);


    }


}
