
package org.firstinspires.ftc.teamcode.autons;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;

@Autonomous(name = "Pedro testington", group = "Autonomous")
@Configurable // Panels
public class UpdatedPedro extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private NanoTimer pathTimer = new NanoTimer();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.886, 125.047, Math.toRadians(38)));

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
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121.886, 125.047),

                                    new Pose(72.971, 83.418)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(38))

                    .build();

            line2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.971, 83.418),

                                    new Pose(101.823, 83.854)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))

                    .build();

            line3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.823, 83.854),

                                    new Pose(129.775, 84.354)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            line4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.775, 84.354),

                                    new Pose(72.125, 78.114)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))

                    .build();

            line5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.125, 78.114),

                                    new Pose(99.577, 59.646)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))

                    .build();

            line6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.577, 59.646),

                                    new Pose(129.026, 58.898)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            line7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.026, 58.898),

                                    new Pose(71.127, 75.369)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))

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
                    follower.followPath(paths.line2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line7);
                    setPathState(7);
                }
                break;


        }

        return pathState;
    }

    public void setPathState(int pState) {

        pathState = pState; pathTimer.resetTimer(); }

}






    