package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel extends SubsystemBase {

    private DcMotorEx flywheel;


    public Flywheel(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "fly");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void periodic() {

        flywheel.setPower(-0.2);


    }


    public void manual (double power) {

        flywheel.setPower(power);


    }


}