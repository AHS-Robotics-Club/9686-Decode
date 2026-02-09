package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;

public class Turret extends SubsystemBase {

    private final DcMotorEx turret;
    private final int COUNTS_PER_REVOLUTION = 1180;
    private double visionOutput;

    private double adjustment;

//    private float position = 0;

    // --- TUNING ---
    // Make these public static so Dashboard can see them
    @Config
    public static class TurretParams {
        public static double kP = 0.025; // Increased for responsiveness
        public static double kI = 0.0;
        public static double kD = 0.0015;

        public static double kStatic = 0.11; // Minimum power to move (friction kick)
        public static double kHeading = 0.01; // Feedforward from chassis rotation

        public static double FILTER_ALPHA = 0.8;
        public static double DEADBAND = 0.5;
        public static double MAX_POWER = 1.0;

        // Encoder / Hardware Constants
        public static double TICKS_PER_REV = 537.7; // GoBilda 312 RPM Series
        public static double GEAR_RATIO = 1.0; // Assume 1:1 for now, adjust if external gears exist
        public static double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

        // Soft Limits (Ticks) - Calibrate these!
        public static int MAX_TICKS = 1500; // Approx 180 degrees right
        public static int MIN_TICKS = -1500; // Approx 180 degrees left




    }

    private PIDController turretPID = new PIDController(0.01,0,0.0000001);

    private PIDController limelightPID = new PIDController(0.015, 0, 0.0000001);

    private double limelightTx = 0.0;
    private boolean hasVisionTarget = false;

    private double lastTx = 0;
    private double lastTargetFieldHeading = 0;
    private boolean hasLastKnown = false;

    double output = 0;
    double angle = 0;
    double position = 0;
    double currentEncoderTicks = 0;

    private double goalDisplacementX = 0;
    private double goalDisplacementY = 0;

    private Pose currPose = new Pose(0, 0, 0);

    private Pose targetPose = new Pose(49, 58, Math.toRadians(90));


    public static double rightAngleLimit = 100;

    public static double leftAngleLimit = -120;



    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders on Init so we know where 0 is (Assuming Front Facing)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize PID
//        controller = new PIDController(TurretParams.kP, TurretParams.kI, TurretParams.kD);
    }



    // Overload for legacy calls or stationary testing without IMU



    private double ticksToDegrees(double ticks) {
        return ticks / TurretParams.TICKS_PER_DEGREE;
    }

    public void stop() {
        turret.setPower(0);
    }

    public void manual(double power) {
        // Apply limits to manual too
        double currentTicks = turret.getCurrentPosition();
        if (power > 0 && currentTicks > TurretParams.MAX_TICKS)
            power = 0;
        if (power < 0 && currentTicks < TurretParams.MIN_TICKS)
            power = 0;
        turret.setPower(power);
    }

    public int getPos() {
        return turret.getCurrentPosition();
    }
    public void autoAim(double tx) {}

    public void spinRight() {}
    public void spinLeft() {}



    public float getAngle() {

        return (((turret.getCurrentPosition()/(float)(COUNTS_PER_REVOLUTION)) * 360) % 360);

    }


    public int getPosition() {

        return turret.getCurrentPosition();

    }


    public void poseGetter(Pose robotPose) {

        currPose = robotPose;




    }

    public void updateVision(double tx, boolean hasTarget) {
        limelightTx = tx;
        hasVisionTarget = hasTarget;
    }


    public void displacementCalc() {

        goalDisplacementX = Math.abs((targetPose.getX()) - currPose.getX());
        goalDisplacementY = Math.abs((targetPose.getY()) - currPose.getY());

    }

    public double targetRotation() {

        return position;


    }

    public void setTargetPoseToMotif() {

        targetPose = RobotConstraints.motifPose;


    }

    public void setTargetPoseToRed() {

        targetPose = RobotConstraints.redGoal;




    }

    public void fineRight() {
        double currX = targetPose.getX();
        double currY = targetPose.getY();
        double currHeading = targetPose.getHeading();

//        targetPose = new Pose(currX + 1, currY - 1, currHeading);

        adjustment += 1.5;
    }

    public void fineLeft() {
        double currX = targetPose.getX();
        double currY = targetPose.getY();
        double currHeading = targetPose.getHeading();

//        targetPose = new Pose(currX - 1, currY -1, currHeading);

        adjustment -= 1.5;
    }

    public void setTargetPoseToBlue() {

        targetPose = RobotConstraints.blueGoal;


    }







    public void periodic() {



        if (currPose == null) return;

        currentEncoderTicks = turret.getCurrentPosition();

        angle = (((((currentEncoderTicks/(double)(COUNTS_PER_REVOLUTION)) * 360))));

        displacementCalc();

        position = Math.toDegrees(Math.atan2(goalDisplacementY, goalDisplacementX)) - Math.toDegrees(currPose.getHeading());

        if (angle > rightAngleLimit) {
            angle += 360;


//            if (angle < leftAngleLimit) {
//
//                angle = leftAngleLimit;
//            }
        }

        if (angle < leftAngleLimit) {
            angle -= 360;


//            if (angle > rightAngleLimit) {
//
//                angle = rightAngleLimit;
//            }
        }

        if (hasVisionTarget) {
            // Optional: low-pass filter the tx to smooth jitter
            // Limelight PID: positive tx means target is right, so turret should move right
            visionOutput = Range.clip(-limelightPID.calculate(0, limelightTx), -0.8, 0.8 ); // target=0, measurement=tx
        } else visionOutput = 0.0;

        output = turretPID.calculate(angle, Range.clip(-position + adjustment, leftAngleLimit, rightAngleLimit));

        turret.setPower(output + visionOutput);




    }
}