
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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private NanoTimer pathTimer;




    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        pathTimer = new NanoTimer();

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
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.000, 123.000),

                                    new Pose(87.000, 83.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(37.5))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.000, 83.000),
                                    new Pose(101.997, 83.624),
                                    new Pose(104.291, 83.749)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(104.291, 83.749),
                                    new Pose(104.828, 83.753),
                                    new Pose(105.364, 83.756),
                                    new Pose(105.904, 83.769),
                                    new Pose(106.409, 83.692),
                                    new Pose(107.154, 84.234),
                                    new Pose(106.580, 81.370),
                                    new Pose(112.032, 94.072),
                                    new Pose(94.073, 46.290),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(144.000, 144.000),
                                    new Pose(0.000, 0.000),
                                    new Pose(115.068, 73.423),
                                    new Pose(118.794, 83.854)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.794, 83.854),

                                    new Pose(84.650, 83.033)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37.5))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.650, 83.033),
                                    new Pose(87.321, 59.733),
                                    new Pose(93.303, 59.116),
                                    new Pose(103.711, 59.650)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103.711, 59.650),

                                    new Pose(118.229, 59.889)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.229, 59.889),

                                    new Pose(81.726, 81.719)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37.5))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.726, 81.719),
                                    new Pose(86.600, 56.402),
                                    new Pose(103.071, 34.690)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103.071, 34.690),

                                    new Pose(118.061, 35.066)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.061, 35.066),

                                    new Pose(78.846, 78.858)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37.5))

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
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    // End of autonomous
                    setPathState(-1);
                }
                break;
        }

        return pathState;
    }

    public void setPathState(int pState) {

        pathState = pState; pathTimer.resetTimer(); }

}
    