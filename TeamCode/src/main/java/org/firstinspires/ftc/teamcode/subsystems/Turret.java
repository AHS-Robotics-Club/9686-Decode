package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private DcMotorEx turret;

    // Tunable auto-aim scaling
    private static final double AUTO_AIM_K = 0.06;   // adjust 0.02–0.05
    private static final double MAX_POWER = 0.5;     // safety cap
    private static final double DEADBAND = .5;      // degrees

    public Turret(HardwareMap hardwareMap) {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {



    }

    // --------------------
    // MANUAL CONTROL
    // --------------------

    public void spinRight() {
        turret.setPower(0.5);
    }

    public void spinLeft() {
        turret.setPower(-0.5);
    }

    public void stop() {
        turret.setPower(0);
    }

    // --------------------
    // AUTO AIM (NO PID)
    // --------------------

    public void autoAim(double tx) {

        // Deadband: already centered
        if (Math.abs(tx) < DEADBAND) {
            stop();
            return;
        }

        // Multiply error by simple gain
        double power = AUTO_AIM_K * tx;

        // Clamp output power
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // Apply power (reverse sign if turret directions are inverted)
        turret.setPower(power);
    }
}
