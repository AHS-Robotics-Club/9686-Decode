package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class OuttakeColorSensor extends SubsystemBase {

    private NormalizedColorSensor outtakeColor;
    public float[] hsvValues = new float[3];
    NormalizedRGBA colors;

    public OuttakeColorSensor (HardwareMap hardwareMap) {

        this.outtakeColor = hardwareMap.get(NormalizedColorSensor.class, "outtakeColor");
        this.colors = outtakeColor.getNormalizedColors();


    }


    public void periodic() {
        colors = outtakeColor.getNormalizedColors();
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
