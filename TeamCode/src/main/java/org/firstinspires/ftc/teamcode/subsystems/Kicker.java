package org.firstinspires.ftc.teamcode.subsystems;

//ball on kicker at 0.707

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Kicker {

    private ServoImplEx kicker;
    private ElapsedTime kickTimer;
    private boolean isKicking = false;

    public Kicker (HardwareMap hardwareMap) {
        this.kicker = hardwareMap.get(ServoImplEx.class, "kicker"); //Expansion 5
        this.kickTimer = new ElapsedTime();
    }

    public void kick() {
            kicker.setPosition(1);
    }

    public void down() {
        kicker.setPosition(0);
    }




    public double getPos() {
        return kicker.getPosition();
    }
}