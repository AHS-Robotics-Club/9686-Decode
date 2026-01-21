package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.AlwaysTrackCommand;
import org.firstinspires.ftc.teamcode.commands.EmptyChamberCommand;
import org.firstinspires.ftc.teamcode.commands.MotifCommand;
import org.firstinspires.ftc.teamcode.commands.TimedKickCommand;
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

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Nice Vision Test Op Mode")
public class PPVisionOpmode extends CommandOpMode {


    private GamepadEx driverPad, gunnerPad;
    private Spindex spindex;
    private Kicker kicker;
    private Intake intake;
    private Turret turret;
    private Flywheel flywheel;
    private Hood hood;

    private Limelight limelight;

    private int totalBalls;
    private int tagID = 0;

    private boolean motifIDdetected = false;

    private boolean lastGreen, lastPurple;

    private boolean hasMotif;
    private boolean wasCalled = false;

    private NanoTimer cycleTimer;

    private IMU imu;



    private OuttakeColorSensor outtakeColor;

    private IntakeColorSensor intakeCD;

    private enum IntakeState {
        IDLE,
        READING,
        INDEXING,
        CONFIRM,
        WAIT_CLEAR
    }

    private IntakeState intakeState = IntakeState.IDLE;

    private boolean pendingGreen = false;
    private boolean pendingPurple = false;

    private int numGreenBalls = 0;
    private int numPurpleBalls = 0;

    private NanoTimer intakeTimer = new NanoTimer();

    public char[] ppg = {'P', 'P', 'G'};
    public char[] pgp = {'P', 'G', 'P'};
    public char[] gpp = {'G', 'P', 'P'};
    public char[] motif = {};



    /* ===================== Shoot FSM ===================== */
    private enum ShootState {
        IDLE,
        ALIGNING,
        KICKING,
        WAIT_CLEAR
    }

    private ShootState shootState = ShootState.IDLE;
    private NanoTimer readingTimer = new NanoTimer();

    private boolean shootRequested = false;
    private boolean isShooting = false;

    /* ===================== Tunables ===================== */

    // Intake
    private static final double INTAKE_BALL_DISTANCE = 50.0;
    private static final double GREEN_THRESHOLD = 0.0145;
    private static final double SPINDEX_BIG_TIME = 0.28;

    // Shooting
    private static final double OUTTAKE_BALL_DISTANCE = 40.0;
    private static final double SPINDEX_STEP_TIME = 0.22;
    private static final double KICK_TIME = 0.12;

    @Override
    public void initialize() {

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());
        follower.startTeleopDrive();
        follower.update();

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.setPollRateHz(100);
//        limelight.pipelineSwitch(0);
//        limelight.start();


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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

        driverPad = new GamepadEx(gamepad1);
        gunnerPad = new GamepadEx(gamepad2);

        hasMotif = (numGreenBalls == 1) && (numPurpleBalls == 2);

//        turret.setDefaultCommand(new AlwaysTrackCommand(turret, limelight));

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            kicker.down();
            spindex.stepForward();
            intake.cycle();
        });

        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            kicker.down();
            spindex.bigStepForward();
            intake.cycle();
        });

        gunnerPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            kicker.down();
            spindex.bigStepForward();
            intake.cycle();
        });
        gunnerPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            kicker.down();
            spindex.stepForward();
            intake.cycle();
        });

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> scheduleMotif());



//        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
//
//            spindex.motif(spindex, kicker, outtakeColor);
//                });





        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new TimedKickCommand(kicker));


        gunnerPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(kicker::kick);
        gunnerPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(kicker::down);




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
    }

    @Override
    public void run() {
        super.run();

        spindex.tagIDgetter(tagID);
        spindex.motifPossible(numPurpleBalls, numGreenBalls);

        hasMotif = (numGreenBalls == 1) && (numPurpleBalls == 2);

        totalBalls = numGreenBalls + numPurpleBalls;

        double distance = intakeCD.getDistance();

        boolean ballPresent = distance < 50;

        boolean isGreen = ballPresent && intakeCD.getGreen() > 0.0145;
        boolean isPurple = ballPresent && !isGreen;


        boolean counting = false;
        boolean pendingGreen = false;
        boolean pendingPurple = false;




//        double flypwr = gamepad2.left_stick_y * -0.85;


        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        LLResult result = limelight.getRawResult();

        boolean hasLL = limelight.hasTarget();
        telemetry.addData("Has LL", hasLL);




        if (gamepad1.options) {
            imu.resetYaw();
        }

        if (gamepad1.dpad_up) intake.expel();

        if (gamepad1.right_bumper) {
            spindex.fineRight();
        } else if (gamepad1.left_bumper) {
            spindex.fineLeft();
        }


        // Pipeline switching

        if (gamepad1.y) {
            limelight.switchPipelineRed();
        }

        if (gamepad1.b) {
            limelight.switchPipelineBlue();
        }


        if (gamepad1.left_trigger != 0) {
            isShooting = true;
        } else if (gamepad1.left_trigger == 0) {
            isShooting = false;
        }



//        if (tagID == 21 && hasMotif && !wasCalled) {
//            motif = gpp;
//            driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new MotifCommand(spindex, kicker, outtakeColor, gpp));
//         //   driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> new MotifCommand(spindex, kicker, outtakeColor, gpp)));
//            wasCalled = true;
//        } else if (tagID == 22 && hasMotif && !wasCalled) {
//            motif = pgp;
//  //          driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> new MotifCommand(spindex, kicker, outtakeColor, pgp)));
//            driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new MotifCommand(spindex, kicker, outtakeColor, pgp));
//            wasCalled = true;
//        } else if (tagID == 23 && hasMotif && !wasCalled) {
//            motif = ppg;
// //           driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> new MotifCommand(spindex, kicker, outtakeColor, ppg)));
//           driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new MotifCommand(spindex, kicker, outtakeColor, ppg));
//            wasCalled = true;
//        } else {
// //           driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(() -> new EmptyChamberCommand(kicker, spindex, outtakeColor)));
//           driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new EmptyChamberCommand(kicker, spindex, outtakeColor));
//            wasCalled = true;
//        }

        updateIntakeFSM();


        flywheel.flyWheelVelo(gamepad1.left_trigger, totalBalls);


        if (gamepad2.dpad_right) {
            turret.spinRight();
        } else if (gamepad2.dpad_left) {
            turret.spinLeft();
        } else turret.stop();






        if (result != null) {

            flywheel.farZone((double)gamepad1.left_trigger, totalBalls, result.getTa());

//            if (hasLL) {
//                turret.autoAim(result.getTx());
//            } else turret.stop();


//                if (gamepad1.dpad_right) turret.spinRight();
//                else if (gamepad1.dpad_left) turret.spinLeft();
//                else turret.stop();
//            }
//        } else {
//            // If result is null, stop the turret for safety/no target
//            turret.stop();
        }






// Hoodlime (Manual Control)
        if (gamepad2.dpad_up) hood.spinUp();
        else if (gamepad2.dpad_down) hood.spinDown();
        else hood.stop();

// Kicker (Manual Control)
//        if (gamepad2.a) kicker.kick();
//        else kicker.down();


//        if (gamepad1.right_trigger != 0) kicker.kick();
//        else kicker.down();

        if (gamepad1.left_trigger != 0 && gamepad1.dpad_left && totalBalls > 0) {

            totalBalls = 0;
            numGreenBalls = 0;
            numPurpleBalls = 0;
        }

        if (hasLL && result != null) {


            telemetry.addData("Pipeline Index", result.getPipelineIndex());
            telemetry.addData("LL VALID", true);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    telemetry.addData(
                            "AprilTag",
                            "ID: %d | Tx: %.2f | Ty: %.2f",
                            fiducial.getFiducialId(),
                            result.getTx(),
                            result.getTy()
                    );

                    if (!motifIDdetected) {
                        tagID = fiducial.getFiducialId();

                        motifIDdetected = true;
                        limelight.switchPipelineRed();

                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

        }



        wasCalled = false;




// Limelight Telemetry (Safely Checked)
        if (hasLL && result != null) {
            telemetry.addData("Pipeline Index", result.getPipelineIndex());
//            telemetry.addData("Apriltag ID", fiducials.get);
            telemetry.addData("LL VALID", true);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("hasLL", hasLL);
            // Safe check for getBotpose() as well
            telemetry.addData("Botpose", result.getBotpose() != null ? result.getBotpose().toString() : "N/A");
        } else {
            telemetry.addLine("Limelight sees nothing / invalid result");
            if (result == null) {
                telemetry.addLine("Limelight result is NULL (Waiting for first valid frame)");
            }
        }

// Spindex Telemetry
        telemetry.addData("Spindex Encoder", spindex.getCurrentPos());
        telemetry.addData("Target Spindex Position", spindex.getPidTarget());
        // telemetry.addData("Flypower", flypwr);
        telemetry.addData("Velocity", flywheel.getVelocity());



        telemetry.addData("tagID", tagID);
        telemetry.addData("Motif based on obelisk", motif);
        telemetry.addData("Total Balls", totalBalls);
        telemetry.addData("G Balls", numGreenBalls);
        telemetry.addData("P Balls", numPurpleBalls);
        telemetry.addData("Has Motif", hasMotif);



        //Color Sensors


        colorTelemetry();


        telemetry.update();



    }




    public void colorTelemetry() {
        telemetry.addData("Intake Red", intakeCD.getRed());
        telemetry.addData("Intake Blue", intakeCD.getBlue());
        telemetry.addData("Intake Green", intakeCD.getGreen());

        telemetry.addData("Intake Hue", intakeCD.getHue());
        telemetry.addData("Intake Saturation", intakeCD.getSat());
        telemetry.addData("Intake Value", intakeCD.getVals());

        telemetry.addData("Intake Distance", intakeCD.getDistance());



        telemetry.addData("Outtake Red", outtakeColor.getRed());
        telemetry.addData("Outtake Blue", outtakeColor.getBlue());
        telemetry.addData("Outtake Green", outtakeColor.getGreen());

        telemetry.addData("Outtake Hue", outtakeColor.getHue());
        telemetry.addData("Outtake Saturation", outtakeColor.getSat());
        telemetry.addData("Outtake Value", outtakeColor.getVals());

        telemetry.addData("Outtake Distance", outtakeColor.getDistance());

        telemetry.addData("lefttriggerfatty", gamepad2.left_trigger);

    }


    private void updateIntakeFSM() {
        if (isShooting) return; // BLOCK intake while shooting

        double distance = intakeCD.getDistance();
        double hue = intakeCD.getHue();

        boolean ballPresent = distance > 5 && distance < RobotConstraints.INTAKE_BALL_CHAMBERED_DISTANCE;
        boolean isGreen = ballPresent && hue < RobotConstraints.OUTTAKE_GREEN_BALL_HUE_THRESH;

        switch (intakeState) {
            case IDLE:
                if (ballPresent && totalBalls() < 3) {


                    readingTimer.resetTimer();
                    intakeState = IntakeState.READING;


                }
                break;


            case READING:

                pendingGreen = isGreen;
                pendingPurple = !isGreen;

                if (readingTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.INTAKE_CD_READING_TIME) {
                    spindex.bigStepForward();
                    cycleTimer.resetTimer();


                    intakeState = PPVisionOpmode.IntakeState.INDEXING;

                }

                break;



            case INDEXING:
                if (cycleTimer.getElapsedTime() >= RobotConstraints.SPINDEX_120_DEG_ROT_TIME) {
                    intakeState = PPVisionOpmode.IntakeState.CONFIRM;
                }
                break;

            case CONFIRM:
                if (pendingGreen) numGreenBalls++;
                if (pendingPurple) numPurpleBalls++;

                pendingGreen = false;
                pendingPurple = false;

                intakeState = PPVisionOpmode.IntakeState.WAIT_CLEAR;
                break;

            case WAIT_CLEAR:
                if (!ballPresent) {
                    intakeState = PPVisionOpmode.IntakeState.IDLE;
                }
                break;
        }
    }

    private int totalBalls() {
        return numGreenBalls + numPurpleBalls;
    }

    private void scheduleMotif() {
        if (!hasMotif) {
            schedule(new EmptyChamberCommand(kicker, spindex, outtakeColor));
            return;
        }

        switch (tagID) {
            case 21:
                schedule(new MotifCommand(spindex, kicker, outtakeColor, gpp));
                break;
            case 22:
                schedule(new MotifCommand(spindex, kicker, outtakeColor, pgp));
                break;
            case 23:
                schedule(new MotifCommand(spindex, kicker, outtakeColor, ppg));
                break;
            default:
                schedule(new EmptyChamberCommand(kicker, spindex, outtakeColor));
        }
    }





}
