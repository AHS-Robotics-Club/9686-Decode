package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spindex extends SubsystemBase {

    private DcMotorEx spindexMtr;

    // REV Through Bore
    private static final int COUNTS_PER_REV = 8192;

    // 1/6 rotation step for manual PID stepping
    private static final double COUNTS_PER_SLOT = COUNTS_PER_REV / 6.0;
    private static final double BIGCOUNTS_PER_SLOT = (int)(COUNTS_PER_REV / 3.0);

    // The actual six slot encoder positions
    private final int[] positions = {
            (int)(0.111 * COUNTS_PER_REV), // 910
            (int)(0.270 * COUNTS_PER_REV), // 2213
            (int)(0.428 * COUNTS_PER_REV), // 3506
            (int)(0.609 * COUNTS_PER_REV), // 4991
            (int)(0.747 * COUNTS_PER_REV), // 6121
            (int)(0.942 * COUNTS_PER_REV)  // 7720
    };

    // Track which of the six indexes you're on
    private int currentIndex = 0;

    // PID for manual fine movement
    private PIDController spindexPID = new PIDController(0.0003207, 0.00, 0.0000141);
    private int pidTarget = 0;

    public Spindex(HardwareMap hardwareMap) {
        spindexMtr = hardwareMap.get(DcMotorEx.class, "spindex");

        spindexMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Keep forward direction predictable
        spindexMtr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // -------------------------
    // PRESET SLOT ROTATION
    // -------------------------

    public void goToIndex(int index) {
        if (index < 0 || index >= positions.length) return;

        currentIndex = index;

        spindexMtr.setTargetPosition(positions[index]);
        spindexMtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexMtr.setPower(0.4);
    }

    public void nextIndex() {
        currentIndex = (currentIndex + 1) % positions.length;
        goToIndex(currentIndex);


    }

    public void previousIndex() {
        currentIndex = (currentIndex - 1 + positions.length) % positions.length;
        goToIndex(currentIndex);
    }

    public void setPidValues(double kP, double kI, double kD) {

        spindexPID.setPID(kP, kI, kD);
    }

    // -------------------------
    // MANUAL PID SLOT STEPPING`
    // -------------------------

    public void stepForward() {
//        pidTarget = -spindexMtr.getCurrentPosition() + (int)COUNTS_PER_SLOT;

        pidTarget += (int)COUNTS_PER_SLOT;

    }

    public void bigStepForward() {
//        pidTarget = -spindexMtr.getCurrentPosition() + (int)BIGCOUNTS_PER_SLOT;

        pidTarget += (int)BIGCOUNTS_PER_SLOT;

    }

    public void stepBackward() {
        pidTarget = spindexMtr.getCurrentPosition() - (int)COUNTS_PER_SLOT;
        spindexMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {

        double current = -spindexMtr.getCurrentPosition();
        double power = spindexPID.calculate(current, pidTarget);
        spindexMtr.setPower(power);

    }

    // -------------------------
    // MANUAL CONTROL
    // -------------------------

    public void manual(double power) {
        spindexMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexMtr.setPower(power);
    }

    public void stop() {
        spindexMtr.setPower(0);
    }

    // -------------------------
    // TELEMETRY
    // -------------------------

    public int getCurrentPos() {
        return spindexMtr.getCurrentPosition();
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public int getPidTarget() {
        return pidTarget;
    }

    public void fineRight() {

        pidTarget += 40;
    }

    public void setPidTarget(int newTarget) {
        pidTarget = newTarget;
    }

    public void fineLeft() {

        pidTarget -= 40;
    }
}


