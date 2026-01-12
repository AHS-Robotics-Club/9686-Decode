package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private final DcMotorEx turret;

    // --- TUNING ---

    // KP: Lowered slightly to reduce the "springiness" / wobble
    private static final double KP = 0.0150;

    // MIN_POWER: CRITICAL FIX.
    // 0.14 was likely too high, causing it to jump past the target.
    // Lowered to 0.07 (enough to move, but not enough to slam).
    private static final double MIN_POWER = 0.11;

    // MAX_POWER: Capped to prevent violence during large turns
    private static final double MAX_POWER = 0.95;

    // DEADBAND: Kept at 1.0 degree.
    // If it still oscillates, increase this to 1.5.
    private static final double DEADBAND = 0.6;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // NOTE: You can handle direction flip here OR in the math below.
        // If the code below doesn't fix the "wrong way" spin, uncomment this line:
        // turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void autoAim(double tx) {
        // 1. Precision Check
        if (Math.abs(tx) < DEADBAND) {
            stop();
            return;
        }

        // 2. Calculate Power
        // FIX FOR "SPINS OPPOSITE":
        // I added a negative sign (-tx).
        // If Limelight says target is to the Right (+tx), we need to turn Right.
        // If your motor needs Negative power to turn Right, this fixes it.
        // If it STILL spins the wrong way, remove the negative sign.
        double power = tx * KP;

        // 3. Friction Feedforward (The "Kick")
        // Only apply the kick if we are actually trying to move
        if (Math.abs(power) > 0.01) {
            if (power > 0) {
                power += MIN_POWER;
            } else {
                power -= MIN_POWER;
            }
        }

        // 4. Safety Clamp
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // 5. Apply
        turret.setPower(power);
    }

    public void stop() {
        turret.setPower(0);
    }

    // Manual overrides
    public void spinRight() { turret.setPower(0.5); }
    public void spinLeft() { turret.setPower(-0.5); }
    public void manual(double power) { turret.setPower(power); }

}