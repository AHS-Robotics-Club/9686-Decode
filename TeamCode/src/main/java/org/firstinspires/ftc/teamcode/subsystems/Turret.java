package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.HardwareBuffer;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends SubsystemBase {

    private DcMotorEx turret;


    public Turret (HardwareMap hardwareMap) {

        turret = hardwareMap.get(DcMotorEx.class, "spindex");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void periodic() {



    }


    public void spinRight() {

        turret.setPower(0.5);



    }

    public void spinLeft() {

        turret.setPower(-0.5);



    }

}
