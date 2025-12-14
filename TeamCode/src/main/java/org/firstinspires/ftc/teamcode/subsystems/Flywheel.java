package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel extends SubsystemBase {

    private final DcMotorEx flywheel;
    private double commandedPower = -0.4;   // default idle power

    public Flywheel(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "fly");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void periodic() {
        // Always apply whatever power we currently want
        flywheel.setPower(commandedPower);




    }

    /** Manually set power (joystick control) */
    public void manual(double joystick) {

        // If joystick is moved, scale between -0.2 and -1
        if (Math.abs(joystick) > 0.05) {
            commandedPower = -0.4 - (Math.abs(joystick) * 0.8);
            // This maps 0 → -0.2 and 1 → -1.0
        } else {
            commandedPower = -0.4;  // go back to idle when joystick centered
        }
    }
}
