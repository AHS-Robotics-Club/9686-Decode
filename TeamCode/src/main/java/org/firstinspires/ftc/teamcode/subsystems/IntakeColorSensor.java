package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeColorSensor extends SubsystemBase {

    private RevColorSensorV3 intakeCD;
    public float[] hsvValues = new float[3];
    NormalizedRGBA colors;

    public IntakeColorSensor (HardwareMap hardwareMap) {

        this.intakeCD = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        this.colors = intakeCD.getNormalizedColors();

        intakeCD.setGain(8);


    }


    public void periodic() {
        colors = intakeCD.getNormalizedColors();
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
        return intakeCD.getDistance(DistanceUnit.MM);
    }








}
