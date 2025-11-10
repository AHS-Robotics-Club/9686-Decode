package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    private ElapsedTime timer;

    public PIDFController(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        timer = new ElapsedTime();
        timer.reset();
    }

    public double getValue(double error) {
        double dT = timer.seconds();
        double derivative = (error - lastError) / dT;

        // sum all the error over time
        integralSum += (error * dT);

        // return the out value eg. motor power
        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
        timer.reset();

        return out;
    }
}