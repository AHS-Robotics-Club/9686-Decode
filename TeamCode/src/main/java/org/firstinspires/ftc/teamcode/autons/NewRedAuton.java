package org.firstinspires.ftc.teamcode.autons;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Subsystems
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@Autonomous(name = "FSM Auton", group = "Autonomous")
@Configurable
public class NewRedAuton extends OpMode {

    private TelemetryManager panelsTelemetry;

    private Follower follower;
    private Paths paths;

    private int pathState = 0;

    private Kicker kicker;
    private Flywheel flywheel;
    private Spindex spindex;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        kicker = new Kicker(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        spindex = new Spindex(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {

        follower.update();        // REQUIRED — PP update loop
        updateStateMachine();     // Run our autonomous states

        // Telemetry
        Pose p = follower.getPose();
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", p.getX());
        panelsTelemetry.debug("Y", p.getY());
        panelsTelemetry.debug("H", p.getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ---------------------------------------------------------------
    // PP Path Definitions — SAME PATH YOU ALREADY HAD
    // ---------------------------------------------------------------
    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()

                    .addPath(new BezierLine(
                            new Pose(14.983, 127.145),
                            new Pose(71.168, 72.624)
                    ))

                    .setTangentHeadingInterpolation()
                    .setReversed()

                    .build();
        }
    }

    // ---------------------------------------------------------------
    // CLEAN PEDRO-PATHING STATE MACHINE
    // ---------------------------------------------------------------
    public void updateStateMachine() {

        switch (pathState) {

            // ---------------------------------------------------
            // STATE 0 — Start the FIRST path
            // ---------------------------------------------------
            case 0:
                follower.followPath(paths.Path1, true);  // << EXACT PP format
                pathState = 1;
                break;

            // ---------------------------------------------------
            // STATE 1 — Wait for the path to finish
            // ---------------------------------------------------
            case 1:
                if (!follower.isBusy()) {

                    // === Actions when path finishes ===
                    flywheel.manual(-0.85);
                    kicker.kick();
                    spindex.bigStepForward();

                    pathState = 2;
                }
                break;

            // ---------------------------------------------------
            // STATE 2 — End. (Do nothing)
            // ---------------------------------------------------
            case 2:
                break;
        }
    }
}
