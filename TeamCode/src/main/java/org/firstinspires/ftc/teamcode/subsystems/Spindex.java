//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Spindex {
//
//    private DcMotor spindexMtr;
//
//    private final double kP = 0.4;
//    private final double MAX_POWER = 0.3;
//    private final double DEADBAND = 0.04;
//
//
//    // Normalized positions for intake/outtake slots (0.0–1.0)
//    private final double[] positions = {
//            0.270,   // intake slot 1
//            0.609,  // intake slot 2
//            0.942,  // intake slot 3
//            0.111,   // outtake slot 1
//            0.428,   // outtake slot 2
//            0.747    // outtake slot 3
//    };
//
//    private int currentIndex = 0;
//
//    // REV Through Bore encoder counts per revolution
//    private static final double COUNTS_PER_REV = 8192.0;
//
//    public Spindex(HardwareMap hardwareMap) {
//        spindexMtr = hardwareMap.get(DcMotor.class, "spindex");
//        spindexMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    /** Returns normalized position (0.0–1.0) */
//    public double getNormalizedPos() {
//        double counts = spindexMtr.getCurrentPosition() % COUNTS_PER_REV;
//        if (counts < 0) counts += COUNTS_PER_REV; // handle negative counts
//        return counts / COUNTS_PER_REV;
//    }
//
//    /** Returns position in degrees (0–360) */
//    public double getDegrees() {
//        return getNormalizedPos() * 360.0;
//    }
//
//    /** Moves spindex to a specific slot index */
////    public void goToIndex(int index) {
////        if (index < 0 || index >= positions.length) return;
////
////        double target = positions[index];
////        double current = getNormalizedPos();
////        double error = target - current;
////
////        // handle wrap-around
////        if (error > 0.5) error -= 1.0;
////        if (error < -0.5) error += 1.0;
////
////        if (Math.abs(error) < DEADBAND) {
////            spindexMtr.setPower(0);
////            currentIndex = index;
////            return;
////        }
////
////        double power = Math.max(-MAX_POWER, Math.min(MAX_POWER, error * kP));
////        spindexMtr.setPower(power);
////
////    }
//
//    public void goToIndex(int index) {
//        if (index < 0 || index >= positions.length) return;
//
//        int targetCounts = (int)(positions[index] * COUNTS_PER_REV);
//        spindexMtr.setTargetPosition(targetCounts);
//        spindexMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spindexMtr.setPower(MAX_POWER);
//        currentIndex = index;
//    }
//
//
//    /** Manual control */
//    public void manual(double power) {
//        spindexMtr.setPower(power);
//    }
//
//    /** Stop motor */
//    public void stop() {
//        spindexMtr.setPower(0);
//    }
//
//    /** Update currentIndex based on closest slot */
//    public void updateCurrentIndex() {
//        double current = getNormalizedPos();
//        currentIndex = 0;
//        double minDistance = Math.abs(current - positions[0]);
//        for (int i = 1; i < positions.length; i++) {
//            double distance = Math.abs(current - positions[i]);
//            if (distance < minDistance) {
//                minDistance = distance;
//                currentIndex = i;
//            }
//        }
//    }
//
//    /** Get current index */
//    public int getCurrentIndex() {
//        return currentIndex;
//    }
//
//    public void nextPosition() {
//        int nextIndex = (currentIndex + 1) % positions.length;
//        goToIndex(nextIndex);
//    }
//
//    public void previousPosition() {
//        int prevIndex = (currentIndex - 1 + positions.length) % positions.length;
//        goToIndex(prevIndex);
//    }
//}


package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spindex extends SubsystemBase {

    private DcMotorEx spindexMtr;

    // Encoder counts per revolution
    private static final int COUNTS_PER_REV = 8192;
    private int targetPosn = 0;

    // Your 6 positions converted to counts
    private final int[] positions = {
            (int)(0.270 * COUNTS_PER_REV), // intake slot 1
            (int)(0.609 * COUNTS_PER_REV), // intake slot 2
            (int)(0.942 * COUNTS_PER_REV), // intake slot 3
            (int)(0.111 * COUNTS_PER_REV), // outtake slot 1
            (int)(0.428 * COUNTS_PER_REV), // outtake slot 2
            (int)(0.747 * COUNTS_PER_REV)  // outtake slot 3
    };

    public Spindex(HardwareMap hardwareMap) {
        spindexMtr = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spindexMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexMtr.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    private PIDController spindexPID = new PIDController(0.00027, 0.0001, 0.0);

    /** Move to a specific slot index (0–5) */
    public void goToIndex(int index) {
        if (index < 0 || index >= positions.length) return;

        spindexMtr.setTargetPosition(positions[index]);
        spindexMtr.setPower(0.3); // power is the speed it moves toward target
    }

    /** Get current motor position for telemetry */
    public int getCurrentPos() {
        return -spindexMtr.getCurrentPosition();
    }

    /** Check if motor reached target */
    public boolean isBusy() {
        return spindexMtr.isBusy();
    }

//    public int getPosition() {
//
 //        int position = -spindexMtr.getCurrentPosition();
//
//        return position;
//
//    }

    public void nextPos() {

        targetPosn = (int)(-spindexMtr.getCurrentPosition() + (8192.0/6.0));




        //spindexMtr.setPower(0);



    }

    public void periodic() {


            double currentPos = -spindexMtr.getCurrentPosition();
            double output = spindexPID.calculate(currentPos, targetPosn);
            spindexMtr.setPower(output);







    }

    public void manual(double power) {

        spindexMtr.setPower(power);

    }

    public void stop() {
        spindexMtr.setPower(0);
    }

    public int targetPos() {
        return targetPosn;

    }

    public double getMotorPwr() {

        return spindexMtr.getPower();
    }

}
