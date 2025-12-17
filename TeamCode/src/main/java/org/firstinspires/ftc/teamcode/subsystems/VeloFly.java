package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VeloFly extends SubsystemBase {

    private final DcMotorEx flywheel;

    // --- Constants ---
    private static final double TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 1.0;

    // --- Controllers ---


    private double targetTicksPerSec = 0.0;


    private double kS = 0;

    private double kV = 0;

    private PIDFController flyPIDF = new PIDFController(0, 0, 0, 0);
    //private SimpleMotorFeedforward flyFF = new SimpleMotorFeedforward(kS, kV);

    public VeloFly(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "fly");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void periodic() {

        double measuredTicks = -flywheel.getVelocity();

       // double ffOut  = flyFF.calculate(targetTicksPerSec, 0);
        double pidOut = flyPIDF.calculate(measuredTicks, targetTicksPerSec);

        double output = pidOut;


        flywheel.setPower(output);
    }

    // ---------------- SETTERS FOR TUNING ----------------

    public void setTargetRPM(double rpm) {
        targetTicksPerSec = (rpm * TICKS_PER_REV) / 60.0 / GEAR_RATIO;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        flyPIDF.setPIDF(kP, kI, kD, kF);


    }

    public void setFlyGains(double kS, double kV) {

        this.kS = kS;
        this.kV = kV;
    }



//    public void setFeedforward(double kS, double kV, double kA) {
//        flyFF.;
//        ff.setKv(kV);
//        ff.setKa(kA);
//    }

    // ---------------- TELEMETRY HELPERS ----------------

    public double getVelocityRPM() {
        return ( -flywheel.getVelocity() * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public double getTargetRPM() {
        return (targetTicksPerSec * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }
}
