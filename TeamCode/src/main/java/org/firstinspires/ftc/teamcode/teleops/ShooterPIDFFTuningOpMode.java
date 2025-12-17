package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.VeloFly;

@Config
@TeleOp(name = "Flywheel PIDF Tuning OpMode")
public class ShooterPIDFFTuningOpMode extends CommandOpMode {

    private VeloFly velofly;
    private FtcDashboard dash;

    // --- Dashboard Tunables ---
    public static double targetRPM = 3000;

    public static double kS = 0.00;
    public static double kV = 0.0000;
    public static double kA = 0.0;

    public static double kP = 0.0000;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kF = 11.7;

    @Override
    public void initialize() {

        dash = FtcDashboard.getInstance();
        velofly = new VeloFly(hardwareMap);

        register(velofly);
    }

    @Override
    public void run() {
        super.run();

        // Live tuning (SAFE)
        velofly.setTargetRPM(targetRPM);
        velofly.setPIDF(kP, kI, kD, kF);
      //  velofly.setFeedforward(kS, kV);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Target RPM", targetRPM);
        packet.put("Measured RPM", velofly.getVelocityRPM());
        packet.put("Error RPM", targetRPM - velofly.getVelocityRPM());

        packet.put("kS", kS);
        packet.put("kV", kV);
        packet.put("kA", kA);

        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);

        dash.sendTelemetryPacket(packet);
    }
}
