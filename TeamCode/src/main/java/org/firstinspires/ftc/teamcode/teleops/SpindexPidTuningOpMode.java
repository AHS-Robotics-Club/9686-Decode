package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name = "Spindex PID Tuning Opmode")
public class SpindexPidTuningOpMode extends CommandOpMode {

    private Spindex spindex;

    private FtcDashboard dash;

    private Intake intake;

    private GamepadEx driverPad;


    public static double kP = 0.000455; // Default PID constants
    public static double kI = 0.0000000;
    public static double kD = 0.00000;

    public static int pidTarget = 0;

    @Override
    public void initialize() {
        // Initialize the Spindex subsystem

        driverPad = new GamepadEx(gamepad1);
        dash = FtcDashboard.getInstance();
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);

        // Register the Spindex subsystem
        register(spindex);
        register(intake);
    }

    @Override
    public void run() {
        super.run();


        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            spindex.bigStepForward();
            intake.cycle();
        });
        if (gamepad2.a) {
            spindex.stepForward();
        }



        // Read the PID constants from the Dashboard sliders
        // Assuming these sliders are created on the Dashboard with the names "kP", "kI", "kD"// Default to current value if not set

        // Set the updated PID values in the Spindex subsystem
        spindex.setPidValues(kP, kI, kD);

        // Get the current position and the target position
        int currentPosition = spindex.getCurrentPos();
        pidTarget = spindex.getPidTarget();

        // Calculate the PID error
        int error = pidTarget - currentPosition;

        // Send the PID constants and the error to the telemetry

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("PID kP", kP);
        packet.put("PID kI", kI);
        packet.put("PID kD", kD);
        packet.put("Current Position", -currentPosition);
        packet.put("Target Position", pidTarget);
        packet.put("Error", error);
        dash.sendTelemetryPacket(packet);

    }
}