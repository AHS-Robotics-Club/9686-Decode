package org.firstinspires.ftc.teamcode.autons;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "Cmd opmode implemetnation blue farside", group = "3")


public class CommandAutonD extends CommandOpMode {

    // Drive Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Intake Motors


    // Sensors
    private Limelight limelight;
    private IMU imu;

    private Spindex spindex;

    private Intake intake;

    private Turret turret;

    private Kicker kicker;

    private Flywheel flywheel;

    @Override
    public void initialize() {
        // Hardware Maps
        leftFront = hardwareMap.get(DcMotor.class, "fL");
        leftBack = hardwareMap.get(DcMotor.class, "bL");
        rightFront = hardwareMap.get(DcMotor.class, "fR");
        rightBack = hardwareMap.get(DcMotor.class, "bR");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);



        limelight = new Limelight(hardwareMap);
        spindex = new Spindex(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        kicker = new Kicker(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);

        register(spindex);
        register(flywheel);
        register(kicker);
        register(limelight);
        register(intake);

        LLResult result = limelight.getRawResult();

        boolean hasLL = limelight.hasTarget();

        limelight.switchPipelineBlue();





        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));

        waitForStart();




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

//        sleep(1000);
//        move(0.5, 0.5, 0.5, 0.5, 1600);
//        flywheel.manual(-0.88);
//        sleep(2000);
//        kicker.kick();
//        sleep(300);
//        kicker.down();
//        sleep(200);
//        spindex.bigStepForward();
//        sleep(500);
//        flywheel.manual(-0.84);
//        sleep(400);
//        kicker.kick();
//        sleep(500);
//        kicker.down();
//        sleep(400);
//        spindex.bigStepForward();
//        sleep(500);
//        flywheel.manual(-0.80);
//        sleep(1000);
//        kicker.kick();
//        sleep(500);
//        kicker.down();
//        sleep(500);
//        move(-0.5, -0.5, -0.5, -0.5, 400);





        telemetry.addLine("ManualAuton Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Your auton code goes here
    }

    @Override

    public void run() {
        super.run();  // VERY IMPORTANT

        if (isStarted()) {


            schedule(
                    new InstantCommand(() -> move(0.5,0.5,0.5,0.5, 1600))

                            .andThen(new InstantCommand(() -> flywheel.manual(-0.88)))
                            .andThen(new WaitCommand(2000))
                            .andThen(new InstantCommand(() -> kicker.kick()))
                            .andThen(new WaitCommand(300))
                            .andThen(new InstantCommand(() -> kicker.down()))
                            .andThen(new WaitCommand(200))
                            .andThen(new InstantCommand(() -> spindex.bigStepForward()))
                            .andThen(new WaitCommand(500))
                            .andThen(new InstantCommand(() -> flywheel.manual(-0.84)))
                            .andThen(new WaitCommand(400))
                            .andThen(new InstantCommand(() -> kicker.kick()))
                            .andThen(new WaitCommand(500))
                            .andThen(new InstantCommand(() -> kicker.down()))
                            .andThen(new WaitCommand(400))
                            .andThen(new InstantCommand(() -> spindex.bigStepForward()))
                            .andThen(new WaitCommand(500))
                            .andThen(new InstantCommand(() -> flywheel.manual(-0.80)))
                            .andThen(new WaitCommand(1000))
                            .andThen(new InstantCommand(() -> kicker.kick()))
                            .andThen(new WaitCommand(500))
                            .andThen(new InstantCommand(() -> kicker.down()))
                            .andThen(new WaitCommand(500))
                            .andThen(new InstantCommand(() -> move(-0.5,-0.5,-0.5,-0.5, 400)))


            );
        }
    }



    // ----------------------------------------------
    // Limelight Functions
    // ----------------------------------------------




    // ----------------------------------------------
    // Movement Helper
    // ----------------------------------------------

    public void move(double lf, double lr, double rf, double rr, long waitTime) {
        leftFront.setPower(lf);
        leftBack.setPower(lr);
        rightFront.setPower(rf);
        rightBack.setPower(rr);

        sleep(waitTime);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}