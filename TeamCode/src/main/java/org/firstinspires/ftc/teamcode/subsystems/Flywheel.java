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





    private PIDFController flyPIDF = new PIDFController(.4, 0, .000000001, 0.00000);

    public Flywheel(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {

        currentVeloTicks = flywheel.getVelocity();
        output = flyPIDF.calculate(currentVeloTicks, targetVeloTicks);

        flywheel.setPower(-output);


    }

    /** Manually set power (joystick control) */
    public void manual(double joystick) {
        flywheel.setPower(-0.875 * joystick);
    }

    public void flyWheelVelo( double leftTrigger, int numBalls) {

// first handle close zone
        if (leftTrigger != 0 || numBalls == 3) {
            targetVeloTicks = CLOSE_ZONE_TICKS;
        } else { targetVeloTicks = AMBIENT_TICKS;
        }

        // If joystick is moved, scale between -0.2 and -1

    }


    public void farZone( double leftTrigger, int numBalls, double tA) {

// first handle close zone
        if  (numBalls == 3 && (tA >= 0.26 && tA <= 0.45)) {

            targetVeloTicks = FAR_ZONE_TICKS;
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
