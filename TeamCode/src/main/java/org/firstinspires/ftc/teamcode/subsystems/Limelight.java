package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight extends SubsystemBase {

    private final Limelight3A limelight;

    // Store last valid results so you never get nulls
    private LLResult lastResult = null;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Low latency (11ms) update rate
        limelight.start();
        limelight.pipelineSwitch(0);   // AprilTag pipeline
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        // Only save good frames
        if (result != null && result.isValid()) {
            lastResult = result;
        }
    }

    // ============================================================
    //               PUBLIC ACCESS METHODS
    // ============================================================

    public boolean hasTarget() {
        return lastResult != null && lastResult.isValid();
    }

    public double getTx() {
        return hasTarget() ? lastResult.getTx() : 0;
    }

    public double getTy() {
        return hasTarget() ? lastResult.getTy() : 0;
    }

    public Pose3D getBotPose() {
        return hasTarget() ? lastResult.getBotpose() : null;
    }

    public LLResult getRawResult() {
        return lastResult;
    }
}
