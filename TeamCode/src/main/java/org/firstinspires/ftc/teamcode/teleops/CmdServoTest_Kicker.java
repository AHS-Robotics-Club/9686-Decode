package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CmdServoTest_Kicker")
public class CmdServoTest_Kicker extends CommandOpMode {

    private SimpleServo kicker;
    private GamepadEx driverPad;
    private double angle = 180;       // starting angle
    private final double step = 2.0;  // change per button press

    @Override
    public void initialize() {
        // Initialize servo
        kicker = new SimpleServo(hardwareMap, "kicker", 0, 360);
        kicker.turnToAngle(angle);

        // Initialize gamepad wrapper
        driverPad = new GamepadEx(gamepad1);

        // Button bindings to increment/decrement servo angle
        driverPad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            angle += step;
            kicker.turnToAngle(angle);
            telemetry.addData("Current Angle", angle);
            telemetry.update();
        });

        driverPad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            angle -= step;
            kicker.turnToAngle(angle);
            telemetry.addData("Current Angle", angle);
            telemetry.update();
        });

        telemetry.addData("Info", "Press A/B to move kicker. Watch servo strain!");
        telemetry.addData("Current Angle", angle);
        telemetry.update();
    }
}
