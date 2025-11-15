package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemFC;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Moulik Op Mode")
public class MoulikOpMode extends CommandOpMode {
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
        limelight.pipelineSwitch(0);  // Set pipeline to AprilTag detection

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
                }
        );

        driverPad.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
                    spindex.bigStepForward();
                    intake.cycle();
                }
        );

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
        LLResult result = limelight.getLatestResult();

        Pose3D botpose = result.getBotpose();

        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
        follower.update();

        double flypwr = Math.min(-0.2, gamepad1.left_stick_y * -0.85);

        flywheel.manual(flypwr);

        if (gamepad1.dpad_right) {
            turret.spinRight();
        } else if (gamepad1.dpad_left) {
            turret.spinLeft();
        } else turret.stop();

        if (gamepad2.dpad_up) {
            hood.spinUp();
        } else if (gamepad2.dpad_down) {
            hood.spinDown();
        } else hood.stop();

        if (gamepad2.a) {
            kicker.kick();
        } else {
            kicker.down();
        }

        telemetry.addData("Spindex Encoder", spindex.getCurrentPos());
        telemetry.addData("Target Spindex Position", spindex.getPidTarget());
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("Botpose", botpose.toString());
        // telemetry.addData("Motor Power", spindex.getMotorPwr());
        telemetry.update();
    }
}