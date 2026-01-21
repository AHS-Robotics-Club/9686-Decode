package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private final DcMotorEx turret;

    // --- TUNING ---
    // Make these public static so Dashboard can see them
    @Config
    public static class TurretParams {
        public static double kP = 0.025; // Increased for responsiveness
        public static double kI = 0.0;
        public static double kD = 0.0015;

        public static double kStatic = 0.11; // Minimum power to move (friction kick)
        public static double kHeading = 0.01; // Feedforward from chassis rotation

        public static double FILTER_ALPHA = 0.8;
        public static double DEADBAND = 0.5;
        public static double MAX_POWER = 1.0;

        // Encoder / Hardware Constants
        public static double TICKS_PER_REV = 537.7; // GoBilda 312 RPM Series
        public static double GEAR_RATIO = 1.0; // Assume 1:1 for now, adjust if external gears exist
        public static double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

        // Soft Limits (Ticks) - Calibrate these!
        public static int MAX_TICKS = 1500; // Approx 180 degrees right
        public static int MIN_TICKS = -1500; // Approx 180 degrees left
    }

    private PIDController controller;

    private double lastTx = 0;
    private double lastTargetFieldHeading = 0;
    private boolean hasLastKnown = false;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders on Init so we know where 0 is (Assuming Front Facing)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PID
        controller = new PIDController(TurretParams.kP, TurretParams.kI, TurretParams.kD);
    }

    /**
     * Updates the turret state. Safe to call every loop.
     * 
     * @param hasTarget    Whether the Limelight currently sees a target.
     * @param tx           The horizontal offset from Limelight (degrees).
     * @param headingVel   The angular velocity of the robot (degrees/sec) from IMU
     * @param robotHeading The absolute heading of the robot (degrees) from IMU
     */
    public void update(boolean hasTarget, double tx, double headingVel, double robotHeading) {

        double currentTicks = turret.getCurrentPosition();
        double turretHeading = ticksToDegrees(currentTicks);

        double power = 0;

        // Recalculate TicksPerDegree just in case Dashboard changed TicksPerRev
        TurretParams.TICKS_PER_DEGREE = (TurretParams.TICKS_PER_REV * TurretParams.GEAR_RATIO) / 360.0;

        if (hasTarget) {
            // ================== VISUAL TRACKING ==================
            // 1. Calculate Field-Centric Angle of Target
            // FieldHeading = RobotHeading + TurretAngle + VisionError
            lastTargetFieldHeading = robotHeading + turretHeading + tx;
            hasLastKnown = true;

            // 2. Filter Vision Data
            double filteredTx = (TurretParams.FILTER_ALPHA * tx) + ((1 - TurretParams.FILTER_ALPHA) * lastTx);
            lastTx = filteredTx;

            // 3. Visual PID (Lock onto tx=0)
            controller.setPID(TurretParams.kP, TurretParams.kI, TurretParams.kD);
            double pidOut = controller.calculate(0, filteredTx);

            // 4. Chassis Feedforward (Counter-act robot spin)
            // If robot spins RIGHT (+Velocity), Feedforward should push LEFT (-)
            double ff = -headingVel * TurretParams.kHeading;

            power = pidOut + ff;

        } else if (hasLastKnown) {
            // ================== MEMORY TRACKING (Field Centric) ==================
            // We want TurretHeading to be: TargetFieldHeading - RobotHeading
            double targetTurretHeading = lastTargetFieldHeading - robotHeading;

            // FLIP LOGIC / WRAP AROUND (Simple Version)
            // If we are commanding > 180 degrees (e.g. 190),
            // the wire might be safer at -170 if limits allow.

            // For now, just a P-Controller on the angle error
            double errorDegrees = targetTurretHeading - turretHeading;

            // Normalize error to shortest path?
            // No, because of wires, we don't always want shortest path if it crosses the
            // back.
            // Be VERY careful here.

            // Simple Position P-Control
            double holdKp = 0.03;
            power = errorDegrees * holdKp;

        } else {
            power = 0;
        }

        // Static Friction Kick
        if (Math.abs(power) > 0.001) {
            power += Math.signum(power) * TurretParams.kStatic;
        }

        // Clamp
        power = Math.max(-TurretParams.MAX_POWER, Math.min(TurretParams.MAX_POWER, power));

        // Soft Limits Check
        if (power > 0 && currentTicks > TurretParams.MAX_TICKS) {
            power = 0;
        } else if (power < 0 && currentTicks < TurretParams.MIN_TICKS) {
            power = 0;
        }

        turret.setPower(power);
    }

    // Overload for legacy calls or stationary testing without IMU
    public void update(boolean hasTarget, double tx, double headingVel) {
        update(hasTarget, tx, headingVel, 0.0);
    }

    public void update(boolean hasTarget, double tx) {
        update(hasTarget, tx, 0.0);
    }

    private double ticksToDegrees(double ticks) {
        return ticks / TurretParams.TICKS_PER_DEGREE;
    }

    public void stop() {
        turret.setPower(0);
    }

    public void manual(double power) {
        // Apply limits to manual too
        double currentTicks = turret.getCurrentPosition();
        if (power > 0 && currentTicks > TurretParams.MAX_TICKS)
            power = 0;
        if (power < 0 && currentTicks < TurretParams.MIN_TICKS)
            power = 0;
        turret.setPower(power);
    }

    public int getPos() {
        return turret.getCurrentPosition();
    }
    public void autoAim(double tx) {}

    public void spinRight() {}
    public void spinLeft() {}
}