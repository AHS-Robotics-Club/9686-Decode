package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hood extends SubsystemBase {

    private CRServoImplEx hood;


    public Hood(HardwareMap hardwareMap) {

        hood = hardwareMap.get(CRServoImplEx.class, "hood");



    }

    public void periodic() {



    }


    public void spinUp() {

        hood.setPower(1);



    }

    public void spinDown() {

        hood.setPower(-1);



    }

    public void stop() {

        hood.setPower(0);



    }

}
