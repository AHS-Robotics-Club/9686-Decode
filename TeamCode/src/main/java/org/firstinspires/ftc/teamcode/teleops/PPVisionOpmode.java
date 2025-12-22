package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.NanoTimer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.AlwaysTrackCommand;
import org.firstinspires.ftc.teamcode.commands.EmptyChamberCommand;
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

    private boolean lastGreen, lastPurple;

    private NanoTimer cycleTimer;

    private IMU imu;



    private OuttakeColorSensor outtakeColor;

    private IntakeColorSensor intakeCD;

    private enum IntakeState {
        IDLE,
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

    /* ===================== Shoot FSM ===================== */
    private enum ShootState {
        IDLE,
        ALIGNING,
        KICKING,
        WAIT_CLEAR
    }

    private ShootState shootState = ShootState.IDLE;
    private NanoTimer shootTimer = new NanoTimer();

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

        driverPad = new GamepadEx(gamepad1);
        gunnerPad = new GamepadEx(gamepad2);

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

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new TimedKickCommand(kicker));

        driverPad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new EmptyChamberCommand(kicker, spindex, outtakeColor));

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
      //  register(outtakeColor);
        register(intakeCD);
    }

    @Override
    public void run() {
        super.run();

        totalBalls = numGreenBalls + numPurpleBalls;

        double distance = intakeCD.getDistance();

        boolean ballPresent = distance < 50;

        boolean isGreen = ballPresent && intakeCD.getGreen() > 0.0145;
        boolean isPurple = ballPresent && !isGreen;


        boolean counting = false;
        boolean pendingGreen = false;
        boolean pendingPurple = false;




//        double flypwr = gamepad2.left_stick_y * -0.85;


        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
        follower.update();

        LLResult result = limelight.getRawResult();

        boolean hasLL = limelight.hasTarget();
        telemetry.addData("Has LL", hasLL);


        turret.setDefaultCommand(new AlwaysTrackCommand(turret, limelight));

        if (gamepad1.options) {
            imu.resetYaw();
        }

        if (gamepad1.dpad_up) intake.expel();
        if (gamepad1.dpad_right) turret.spinRight();
        else if (gamepad1.dpad_left) turret.spinLeft();
        else turret.stop();

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

        updateIntakeFSM();


        flywheel.flyWheelVelo(gamepad1.left_trigger, totalBalls);








        if (result != null) {

            flywheel.farZone((double)gamepad1.left_trigger, totalBalls, result.getTa());

            if (gamepad2.y && hasLL) {
                turret.autoAim(result.getTx());
            } else {
                if (gamepad1.dpad_right) turret.spinRight();
                else if (gamepad1.dpad_left) turret.spinLeft();
                else turret.stop();
            }
        } else {
            // If result is null, stop the turret for safety/no target
            turret.stop();
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

// Limelight Telemetry (Safely Checked)
        if (hasLL && result != null) {
            telemetry.addData("LL VALID", true);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());
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


        telemetry.addData("Total Balls", totalBalls);
        telemetry.addData("G Balls", numGreenBalls);
        telemetry.addData("P Balls", numPurpleBalls);



        //Color Sensors


        colorTelemetry();


        telemetry.update();



    }

//    public void emptyChamber() {
//
//        // if a ball is detected in the intake threshhold, the ball is not aligned in outtake, so step by 60 degs.
//
//        boolean shot1 = false;
//        boolean shot2 = false;
//        boolean shot3 = false;
//
//        boolean hasBalls = totalBalls > 0;
//
//
//        if (intakeCD.getDistance() < RobotConstraints.INTAKE_BALL_CHAMBERED_DISTANCE) spindex.stepForward();
//
//
//        if (hasBalls) {
//            kicker.kick();
//            kicker.down();
//            shot1 = true;
//
//            totalBalls--;
//
//            spindex.bigStepForward();
//            transferTimer.resetTimer();
//        }
//
//        if (transferTimer.getElapsedTime() > RobotConstraints.SPINDEX_120_DEG_ROT_TIME && shot1 && hasBalls) {
//        kicker.kick();
//        kicker.down();
//
//        totalBalls--;
//
//        spindex.bigStepForward();
//        transferTimer.resetTimer();
//
//        shot2 = true;
//        }
//
//        if (transferTimer.getElapsedTime() > RobotConstraints.SPINDEX_120_DEG_ROT_TIME && shot2 && hasBalls) {
//            kicker.kick();
//            kicker.down();
//
//            totalBalls--;
//
//            spindex.bigStepForward();
//            transferTimer.resetTimer();
//
//            shot3 = true;
//        }
//
//
//
//
//    }


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
        double green = intakeCD.getGreen();

        boolean ballPresent = distance > 5 && distance < RobotConstraints.INTAKE_BALL_CHAMBERED_DISTANCE;
        boolean isGreen = ballPresent && green > GREEN_THRESHOLD;

        switch (intakeState) {
            case IDLE:
                if (ballPresent && totalBalls() < 3) {
                    pendingGreen = isGreen;
                    pendingPurple = !isGreen;

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




}
