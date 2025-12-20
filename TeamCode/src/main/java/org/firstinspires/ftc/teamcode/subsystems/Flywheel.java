package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel extends SubsystemBase {

    private final DcMotorEx flywheel;
    private double commandedPower = -0.4;   // default idle power

    private final double CLOSE_ZONE_TICKS = -1275;
    private final double FAR_ZONE_TICKS = -1500;

    private final double AMBIENT_TICKS = -800;


    private double currentVeloTicks = 0;

    private double targetVeloTicks = 0;

    private double output = 0;





    private PIDFController flyPIDF = new PIDFController(.04, 0, .0001, 0.000008);

    public Flywheel(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        // Always apply whatever power we currently want
       // flywheel.setPower(commandedPower);


        currentVeloTicks = flywheel.getVelocity();
        output = flyPIDF.calculate(-currentVeloTicks, targetVeloTicks);

        flywheel.setPower(output);




    }

    /** Manually set power (joystick control) */
    public void manual(double joystick) {
        flywheel.setPower(-0.875 * joystick);
    }

    public void autoZone(double tA, double leftTrigger) {

// first handle close zone
        if (leftTrigger != 0 && (tA >= 0.61 && tA <= 1.46)) {
            targetVeloTicks = CLOSE_ZONE_TICKS;
        } else if (leftTrigger != 0 && (tA >= 0.26 && tA <= 0.45)) {

            targetVeloTicks = FAR_ZONE_TICKS;
        } else {
            targetVeloTicks = AMBIENT_TICKS;
        }

        // If joystick is moved, scale between -0.2 and -1

    }

    public void setTargetVeloTicks(double pidTarget) {

        targetVeloTicks = pidTarget;

    }



    public double getVelocity() {

        return flywheel.getVelocity();
    }
}
