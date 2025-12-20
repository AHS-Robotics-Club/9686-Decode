package org.firstinspires.ftc.teamcode.subsystems;

//ball on kicker at 0.707

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Kicker extends SubsystemBase {

    private ServoImplEx kicker;

    private NanoTimer kickTimer;

    public Kicker (HardwareMap hardwareMap) {
        this.kicker = hardwareMap.get(ServoImplEx.class, "kicker"); //Expansion 5

        kickTimer = new NanoTimer();
    }

    @Override
    public void periodic() {
        kicker.setPosition(kicker.getPosition());
    }

    public void kick() {
        kicker.setPosition(1);
    }

    public void down() {
        kicker.setPosition(0.49);
    }


    public void timedKick() {



    }




    public double getPos() {
        return kicker.getPosition();
    }
}