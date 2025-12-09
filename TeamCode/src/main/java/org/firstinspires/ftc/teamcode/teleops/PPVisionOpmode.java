package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.hardware.HardwareBuffer;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.AlwaysTrackCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
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

        limelight = new Limelight(hardwareMap);

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);

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
    }

    @Override
    public void run() {
        super.run();

        double flypwr = gamepad2.left_stick_y * -0.88;
        flywheel.manual(flypwr);

        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
        follower.update();

        LLResult result = limelight.getRawResult();

        boolean hasLL = limelight.hasTarget();
        telemetry.addData("Has LL", hasLL);


        turret.setDefaultCommand(new AlwaysTrackCommand(turret, limelight));

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
        if (result != null) {
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
        if (gamepad2.a) kicker.kick();
        else kicker.down();

// Limelight Telemetry (Safely Checked)
        if (hasLL && result != null) {
            telemetry.addData("LL VALID", true);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
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
        telemetry.addData("Flypower", flypwr);

        telemetry.update();

    }
}
