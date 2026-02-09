
package org.firstinspires.ftc.teamcode.autons;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Go forward red goal side", group = "Autonomous")
@Configurable // Panels
public class GoForwardRedGoalSide extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private NanoTimer pathTimer = new NanoTimer();



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(83, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

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
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83, 8.000),

                                    new Pose(83, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();
        }
    }


    public int autonomousPathUpdate() {


        switch (pathState) {

            case 0:
                // Start Path 1
                follower.followPath(paths.Path1);

                setPathState(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    RobotConstraints.teleOpStartPose = follower.getPose();
                }
                break;




            // Add your state machine Here
            // Access paths with paths.pathName
            // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        }

        return pathState;




    }


    public void setPathState (int pState) {

        pathState = pState; pathTimer.resetTimer(); }

}
    