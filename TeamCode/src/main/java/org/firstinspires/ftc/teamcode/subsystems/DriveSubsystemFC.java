package org.firstinspires.ftc.teamcode.subsystems;

import android.inputmethodservice.InputMethodService;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystemFC extends SubsystemBase {

    private Motor fL, fR, bL, bR;

    private IMU imu;

    private double mult;

    private MecanumDrive mDrive;


    public DriveSubsystemFC(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, IMU imu, double multiplier){
        fL = frontLeft;
        fR = frontRight;
        bL = backLeft;
        bR = backRight;

        mult = multiplier;

        this.imu = imu;



        mDrive = new MecanumDrive(fL, fR, bL, bR);
    }


    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed){

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        mDrive.driveFieldCentric(strafeSpeed*mult, forwardSpeed*mult, turnSpeed*mult, heading, false);

    }





}