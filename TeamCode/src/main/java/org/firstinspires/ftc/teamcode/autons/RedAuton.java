package org.firstinspires.ftc.teamcode.autons;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Import the Kicker subsystem
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@Autonomous(name = "Red Auton", group = "Autonomous") // Renamed the OpMode
@Configurable // Panels
public class RedAuton extends OpMode { // Renamed the class

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Declare the Kicker subsystem
    private Kicker kicker;
    private Flywheel flywheel;

    private Spindex spindex;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Note: You may want to change this starting pose for the Red alliance
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        // Initialize the Kicker subsystem
        kicker = new Kicker(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        spindex = new Spindex(hardwareMap);

        // pathState is 0 by default, which is our starting state

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            // Note: You may want to change these coordinates for the Red alliance
                            new BezierLine(new Pose(123.607, 122.358), new Pose(72.000, 71.792))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(36))
                    .setReversed()
                    .build();
        }
    }

    /**
     * This is the state machine for our autonomous path.
     * It returns the new state, which is then assigned to the pathState variable in loop().
     */
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // State 0: Start following Path1
                follower.followPath(paths.Path1);

                // Immediately move to state 1 to wait for the path to finish
                return 1;

            case 1:
                // State 1: Wait for the path to complete
                if (!follower.isBusy()) {
                    // Path is finished, so we run our action
                    flywheel.manual(-0.85);
                    kicker.kick();
                    kicker.down();
                    spindex.bigStepForward();


                    // Move to state 2 (the "done" state)
                    return 2;
                }

                // Path is not finished, stay in state 1
                return 1;

            case 2:
                // State 2: Autonomous is finished. Do nothing.
                return 2;

            default:
                // If pathState somehow becomes an unknown value, go to the "done" state
                return 2;
        }
    }
}