package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.RobotConstraints;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Turret Dual PID w Limelight")
public class TurretDualPIDLimelightOpMode extends CommandOpMode {

    private DcMotorEx turret;
    private final int COUNTS_PER_REVOLUTION = 1180;

    private PIDController odomPID = new PIDController(0,0,0);
    private PIDController visionPID = new PIDController(0,0,0);

    // Live-tunable PID gains
    public static double odom_kP = 0.005, odom_kI = 0, odom_kD = 0.0000001;
    public static double vision_kP = 0.01, vision_kI = 0, vision_kD = 0.0000001;

    public static double leftAngleLimit = -140;
    public static double rightAngleLimit = 160;

    private Pose robotPose = new Pose(0,0,0);
    private Pose goalPose = RobotConstraints.redGoal;

    private double angle = 0;
    private double position = 0;
    private double output = 0;
    private double visionOutput = 0;

    private Limelight limelight;
    private IMU imu;

    @Override
    public void initialize() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize Limelight subsystem
        limelight = new Limelight(hardwareMap);

        // Optional IMU for robot heading if using odom PID
        imu = hardwareMap.get(IMU.class, "imu");

        // Start dashboard

    }

    @Override
    public void run() {
        FtcDashboard dash = FtcDashboard.getInstance();

        // Update PID gains live
        odomPID.setPID(odom_kP, odom_kI, odom_kD);
        visionPID.setPID(vision_kP, vision_kI, vision_kD);

        // Current turret angle
        double currentTicks = turret.getCurrentPosition();
        angle = (currentTicks / (double)COUNTS_PER_REVOLUTION) * 360.0;

        // Odometry target angle
        double robotHeading = (imu != null) ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) : 0.0;
        double deltaX = goalPose.getX() - robotPose.getX();
        double deltaY = goalPose.getY() - robotPose.getY();
        position = Math.toDegrees(Math.atan2(deltaY, deltaX)) - robotHeading;

        // Wrap angle within soft limits
        if(angle > rightAngleLimit) angle -= 360;
        if(angle < leftAngleLimit) angle += 360;

        // Odom PID
        output = odomPID.calculate(angle, clamp(position, leftAngleLimit, rightAngleLimit));

        // Vision PID

        boolean hasTarget = limelight.hasTarget();
        double tx = limelight.getTx();

        if(hasTarget) {
            visionOutput = -visionPID.calculate(tx, 0.0);
        } else {
            visionOutput = 0.0;
        }

        // Apply motor power
        turret.setPower(output + visionOutput);

        // Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret Angle", angle);
        packet.put("Odom Target", position);
        packet.put("Vision Tx", tx);
        packet.put("Vision Output", visionOutput);
        packet.put("Odom Output", output);
        packet.put("Final Power", output + visionOutput);
        packet.put("Odom kP", odom_kP);
        packet.put("Odom kI", odom_kI);
        packet.put("Odom kD", odom_kD);
        packet.put("Vision kP", vision_kP);
        packet.put("Vision kI", vision_kI);
        packet.put("Vision kD", vision_kD);
        packet.put("Has Vision Target", hasTarget);

        dash.sendTelemetryPacket(packet);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
