package org.firstinspires.ftc.teamcode.autons;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

@Autonomous(name = "ManualAuton", group = "3")
public class ManualAuton extends LinearOpMode {

    // Drive Motors
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Intake Motors
    private DcMotor intake;
    private DcMotor backIntake;

    // Sensors
    private Limelight3A limelight;
    private IMU imu;

    private Spindex spindex;

    private Kicker kicker;

    private Flywheel flywheel;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Maps
        leftFront = hardwareMap.get(DcMotor.class, "fL");
        leftBack = hardwareMap.get(DcMotor.class, "bL");
        rightFront = hardwareMap.get(DcMotor.class, "fR");
        rightBack = hardwareMap.get(DcMotor.class, "bR");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "it");

        spindex = new Spindex(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        kicker = new Kicker(hardwareMap);



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));

        waitForStart();
        limelight.start();

        sleep(1000);
        move(0.5, 0.5, 0.5, 0.5, 1600);
        flywheel.manual(-0.8);
        sleep(2000);
        kicker.kick();
        sleep(500);
        kicker.down();
        sleep(200);
        spindex.bigStepForward();
        spindex.periodic();
        sleep(500);
        flywheel.manual(-0.7);
        sleep(400);
        kicker.kick();
        sleep(500);
        kicker.down();
        sleep(400);
        spindex.bigStepForward();
        spindex.periodic();
        sleep(500);
        flywheel.manual(-0.65);
        sleep(1000);
        kicker.kick();
        sleep(500);
        kicker.down();
        sleep(500);
        move(-0.5, -0.5, -0.5, -0.5, 400);





        telemetry.addLine("ManualAuton Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Your auton code goes here
    }

    // ----------------------------------------------
    // Limelight Functions
    // ----------------------------------------------

    public double detectX() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("BotPose", result.getBotpose().toString());
            telemetry.addData("Yaw", result.getBotpose().getOrientation());
        }

        return (result != null) ? result.getTx() : 0;
    }

    public double detectY() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            telemetry.addData("Tx", result.getTx());
            telemetry.addData("Ty", result.getTy());
            telemetry.addData("BotPose", pose.toString());
            telemetry.addData("Yaw", pose.getOrientation());
        }

        return (result != null) ? result.getTy() : 0;
    }

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
