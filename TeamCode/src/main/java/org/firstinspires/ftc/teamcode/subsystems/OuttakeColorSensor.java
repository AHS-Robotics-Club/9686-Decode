package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OuttakeColorSensor extends SubsystemBase {

    private RevColorSensorV3 outtakeCD;

    public float[] hsvValues = new float[3];
    NormalizedRGBA colors;

    public OuttakeColorSensor (HardwareMap hardwareMap) {

        this.outtakeCD = hardwareMap.get(RevColorSensorV3.class, "outtakeColor");

        this.colors = outtakeCD.getNormalizedColors();
        outtakeCD.setGain(12);


    }


    public void periodic() {
        colors = outtakeCD.getNormalizedColors();
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

    public double getDistance() {
        return outtakeCD.getDistance(DistanceUnit.MM);
    }










}
