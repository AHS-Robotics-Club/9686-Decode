package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Nice Vision Test Op Mode")
public class PPVisionOpmode extends CommandOpMode {

    private GamepadEx driverPad, gunnerPad;
    private Spindex spindex;
    private Kicker kicker;
    private Intake intake;
    private Turret turret;
    private Flywheel flywheel;
    private Hood hood;

    private Limelight3A limelight;

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);

        driverPad = new GamepadEx(gamepad1);
        gunnerPad = new GamepadEx(gamepad2);

        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            spindex.stepForward();
            intake.cycle();
        });

        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            spindex.bigStepForward();
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
    }

    @Override
    public void run() {
        super.run();

        double flypwr = gamepad2.left_stick_y * -0.85;
        flywheel.manual(flypwr);

        follower.setTeleOpDrive(gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                false);
        follower.update();

        LLResult result = limelight.getLatestResult();

        boolean hasLL = (result != null && result.isValid());

        // -----------------------
        // TURRET AUTO-AIM LOGIC
        // -----------------------
        if (gamepad2.y && hasLL) {
            turret.autoAim(result.getTx());
        } else {
            if (gamepad1.dpad_right) turret.spinRight();
            else if (gamepad1.dpad_left) turret.spinLeft();
            else turret.stop();
        }


        // Pipeline switching

        if (gamepad1.x) {

            limelight.pipelineSwitch(0);

        }

        if (gamepad1.y) {
            limelight.pipelineSwitch(1);
        }



        // Hood
        if (gamepad2.dpad_up) hood.spinUp();
        else if (gamepad2.dpad_down) hood.spinDown();
        else hood.stop();

        // Kicker
        if (gamepad2.a) kicker.kick();
        else kicker.down();

        // -----------------------
        // TELEMETRY
        // -----------------------
        if (hasLL) {
            telemetry.addData("LL VALID", true);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Botpose", result.getBotpose().toString());
        } else {
            telemetry.addLine("⚠️ Limelight sees nothing / invalid result");
        }

        telemetry.addData("Spindex Encoder", spindex.getCurrentPos());
        telemetry.addData("Target Spindex Position", spindex.getPidTarget());

        telemetry.update();
    }
}
