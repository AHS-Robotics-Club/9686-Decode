
package org.firstinspires.ftc.teamcode.autons;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.commands.MotifCommand;
import org.firstinspires.ftc.teamcode.commands.MotifFSM;
import org.firstinspires.ftc.teamcode.commands.SimpleShootFSM;
import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.teleops.PPVisionOpmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Pedro testington", group = "Autonomous")
@Configurable // Panels
public class UpdatedPedro extends CommandOpMode {

    private MotifFSM motifFSM;
    private SimpleShootFSM simpleShootFSM;

    private enum IntakeState {
        IDLE,
        READING,
        INDEXING,
        CONFIRM,
        WAIT_CLEAR
    }

    public char[] ppg = { 'P', 'P', 'G' };
    public char[] pgp = { 'P', 'G', 'P' };
    public char[] gpp = { 'G', 'P', 'P' };

    private boolean isShooting = false;

    private UpdatedPedro.IntakeState intakeState = UpdatedPedro.IntakeState.IDLE;

    private boolean pendingGreen = false;
    private boolean pendingPurple = false;

    private boolean scheduled = false;

    private int numGreenBalls = 0;
    private int numPurpleBalls = 0;

    private int totalBalls = 0;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Turret turret;

    private Spindex spindex;

    private Limelight limelight;
    private Intake intake;
    private Kicker kicker;
    private Flywheel flywheel;
    private Hood hood;
    private IntakeColorSensor intakeCD;
    private OuttakeColorSensor outtakeColor;

    private NanoTimer cycleTimer = new NanoTimer();
    private NanoTimer readingTimer  = new NanoTimer();

    private NanoTimer pathTimer = new NanoTimer();

    public static Pose endingPose;

    @Override
    public void initialize() {



        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.886, 125.047, Math.toRadians(38)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        limelight = new Limelight(hardwareMap);

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        intakeCD = new IntakeColorSensor(hardwareMap);
        outtakeColor = new OuttakeColorSensor(hardwareMap);

        cycleTimer = new NanoTimer();
        readingTimer = new NanoTimer();

        register(flywheel);
        register(spindex);
        register(intake);
        register(turret);
        register(hood);
        register(kicker);
        register(limelight);
        register(turret, limelight);
        register(outtakeColor);
        register(intakeCD);


        motifFSM = new MotifFSM(spindex, kicker, outtakeColor);
        simpleShootFSM = new SimpleShootFSM(spindex, kicker, 3, intakeCD);
    }

    @Override
    public void run() {
        totalBalls = numGreenBalls + numPurpleBalls;
        turret.poseGetter(follower.getPose());

        flywheel.setTargetVeloTicks(-1275);
        super.run();
        updateIntakeFSM();
        motifFSM.update();
        simpleShootFSM.update();
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

                    if (!scheduled) {

                        simpleShootFSM.start();

                        isShooting = true;

                        scheduled = true;

                    }


                    if (pathTimer.getElapsedTime(TimeUnit.MILLISECONDS) > 5000) {
                        follower.followPath(paths.line2);
                        setPathState(2);
                        isShooting = false;
                        scheduled = false;
                        totalBalls = 0;
                        numGreenBalls = 0;
                        numPurpleBalls = 0;
                        spindex.stepForward();


                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line3, 0.35, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {

                    if (!scheduled) {

                        simpleShootFSM.start();

                        isShooting = true;

                        scheduled = true;

                    }


                    if (pathTimer.getElapsedTime(TimeUnit.MILLISECONDS) > 4000) {
                        follower.followPath(paths.line4);
                        setPathState(4);
                        isShooting = false;
                        scheduled = false;
                        totalBalls = 0;
                        numGreenBalls = 0;
                        numPurpleBalls = 0;
                        spindex.stepForward();


                    }
                }

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line6, 0.35, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line7);
                    setPathState(7);

                    if (!follower.isBusy()) {

                        RobotConstraints.teleOpStartPose = follower.getPose();
                    }
                }
                break;


        }

        return pathState;
    }

    private void updateIntakeFSM() {
        if (isShooting)
            return; // BLOCK intake while shooting

        double distance = intakeCD.getDistance();
        double hue = intakeCD.getHue();

        boolean ballPresent = distance > 5 && distance < RobotConstraints.INTAKE_BALL_CHAMBERED_DISTANCE;
        boolean isGreen = ballPresent && hue < RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH;

        switch (intakeState) {
            case IDLE:
                if (ballPresent && totalBalls() < 3) {

                    readingTimer.resetTimer();
                    intakeState = UpdatedPedro.IntakeState.READING;

                }
                break;

            case READING:

                pendingGreen = isGreen;
                pendingPurple = !isGreen;

                if (readingTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.INTAKE_CD_READING_TIME) {
                    spindex.bigStepForward();
                    cycleTimer.resetTimer();

                    intakeState = UpdatedPedro.IntakeState.INDEXING;

                }

                break;

            case INDEXING:
                if (cycleTimer.getElapsedTime() >= RobotConstraints.SPINDEX_120_DEG_ROT_TIME) {
                    intakeState = UpdatedPedro.IntakeState.CONFIRM;
                }
                break;

            case CONFIRM:
                if (pendingGreen)
                    numGreenBalls++;
                if (pendingPurple)
                    numPurpleBalls++;

                pendingGreen = false;
                pendingPurple = false;

                intakeState = UpdatedPedro.IntakeState.WAIT_CLEAR;
                break;

            case WAIT_CLEAR:
                if (!ballPresent) {
                    intakeState = UpdatedPedro.IntakeState.IDLE;
                }
                break;
        }
    }

    private int totalBalls() {
        return numGreenBalls + numPurpleBalls;
    }


    public void setPathState(int pState) {

        pathState = pState; pathTimer.resetTimer(); }

}






    