package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Command Auton", group = "Autonomous")
public class PedroCommandAuton extends CommandOpMode {

    private Follower follower;

    private Spindex spindex;
    private Flywheel flywheel;
    private Kicker kicker;
    private Turret turret;
    private Intake intake;
    private Limelight limelight;

    private PathChain toShootingSpot;
    private PathChain toPark;

    @Override
    public void initialize() {
        // Initialize Subsystems
        spindex = new Spindex(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        kicker = new Kicker(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        limelight.switchPipelineBlue();

        register(spindex, flywheel, kicker, turret, intake, limelight);

        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);

        // Define Start Pose (You need to tune these coordinates)
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        // Build Paths
        buildPaths(startPose);

        // Schedule the Autonomous Routine
        schedule(
                // Drive to shooting spot using Odometry
                new FollowPathCommand(follower, toShootingSpot, true),

                // Shooting Routine
                new InstantCommand(() -> flywheel.setTargetVeloTicks(-1275)),
                new WaitCommand(2000),
                new InstantCommand(() -> kicker.kick()),
                new WaitCommand(300),
                new InstantCommand(() -> kicker.down()),
                new WaitCommand(200),
                new InstantCommand(() -> spindex.bigStepForward()),
                new WaitCommand(500),
                new InstantCommand(() -> flywheel.setTargetVeloTicks(-1275)),
                new WaitCommand(400),
                new InstantCommand(() -> kicker.kick()),
                new WaitCommand(500),
                new InstantCommand(() -> kicker.down()),
                new WaitCommand(400),
                new InstantCommand(() -> spindex.bigStepForward()),
                new WaitCommand(500),
                new InstantCommand(() -> flywheel.setTargetVeloTicks(-1275)),
                new WaitCommand(1000),
                new InstantCommand(() -> kicker.kick()),
                new WaitCommand(500),
                new InstantCommand(() -> kicker.down()),
                new WaitCommand(500),

                // Drive back/park using Odometry
                new FollowPathCommand(follower, toPark, true));

        telemetry.addLine("PedroCommandAuton Initialized");
        telemetry.update();
    }

    private void buildPaths(Pose startPose) {
        // Example Coordinates - TUNE THESE
        Pose shootingPose = new Pose(24, 0, 0); // Move forward 24 inches
        Pose parkPose = new Pose(5, 0, 0); // Move back near start

        toShootingSpot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void run() {
        super.run(); // Runs the scheduled commands
        follower.update(); // Updates Odometry and Path Following

        // Optional: Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
