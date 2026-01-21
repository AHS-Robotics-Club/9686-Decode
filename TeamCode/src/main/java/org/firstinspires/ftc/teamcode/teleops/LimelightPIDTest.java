package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
@TeleOp(name = "Limelight PID Tuning")
public class LimelightPIDTest extends CommandOpMode {

    /* ================= Dashboard Tunables ================= */

    public static double kP = 0.015;
    public static double kI = 0.0;
    public static double kD = 0.001;

    public static double TARGET_TX = 0.0; // usually zero

    /* ================= Hardware ================= */

    private Turret turret;
    private Limelight limelight;

    /* ================= PID ================= */

    private FtcDashboard dashboard;
    private IMU imu;

    @Override
    public void initialize() {

        turret = new Turret(hardwareMap);
        limelight = new Limelight(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        dashboard = FtcDashboard.getInstance();

        register(turret);
        register(limelight);
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.x) {
            limelight.switchPipelineRed();
        } else if (gamepad1.y) {
            limelight.switchPipelineBlue();
        }

        TelemetryPacket packet = new TelemetryPacket();
        boolean hasTarget = limelight.hasTarget();
        packet.put("Has Target", hasTarget);

        // Sync Dashboard PID values to Turret Subsystem
        Turret.TurretParams.kP = kP;
        Turret.TurretParams.kI = kI;
        Turret.TurretParams.kD = kD;

        // Delegate all logic to the Turret subsystem
        double headingVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double tx = hasTarget ? limelight.getTx() : 0;

        turret.update(hasTarget, tx, headingVel, robotHeading);

        packet.put("Mode", hasTarget ? "Tracking" : "Searching");
        /* ================= Telemetry ================= */

        // Always send tx and Target tx for graphing
        packet.put("Error (tx)", hasTarget ? tx : 0);
        packet.put("Target (Setpt)", TARGET_TX);
        // packet.put("Turret Pos (Ticks)", turret.getPos()); // No Encoder

        packet.put("Heading Vel", headingVel);

        packet.put("kP", Turret.TurretParams.kP);
        packet.put("kD", Turret.TurretParams.kD);
        packet.put("kF_Static", Turret.TurretParams.kStatic);
        packet.put("kF_Heading", Turret.TurretParams.kHeading);
        packet.put("Filter Alpha", Turret.TurretParams.FILTER_ALPHA);

        dashboard.sendTelemetryPacket(packet);
    }
}
