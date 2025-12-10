package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class IntakeColorSensor extends SubsystemBase {

    private NormalizedColorSensor intakeColor;
    private DistanceSensor intakeDistance;
    public float[] hsvValues = new float[3];
    NormalizedRGBA colors;

    public IntakeColorSensor (HardwareMap hardwareMap) {

        this.intakeColor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        this.colors = intakeColor.getNormalizedColors();
        this.intakeDistance = hardwareMap.get(DistanceSensor.class, "outtakeColor");


    }


    public void periodic() {
        colors = intakeColor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);



    }

    public float getRed() {

        return colors.red;
    }

    public float getGreen() {

        return colors.green;
    }

    public float getBlue() {

        return colors.blue;
    }

    public float getAlpha() {

        return colors.alpha;
    }

    public float getHue() {

        return hsvValues[0];
    }

    public float getSat() {

        return hsvValues[1];
    }

    public float getVals() {

        return hsvValues[2];
    }








}
