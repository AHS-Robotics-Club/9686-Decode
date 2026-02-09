package org.firstinspires.ftc.teamcode.teleops;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;


@Config
@TeleOp(name = "Turret Testing Opmode")
public class TurretTestOpMode extends CommandOpMode {


    private DcMotorEx turret;

    private final int COUNTS_PER_REVOLUTION = 1180;

    private double goalDisplacementX = 0;
    private double goalDisplacementY = 0;


    private double tx = 0;

    public static double rightAngleLimit = 160;

    public static double leftAngleLimit = -140;

    private FtcDashboard dash;

    public static double position = 0;

    public static double angle = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double viskP = 0;
    public static double viskI = 0;
    public static double viskD = 0;

    private double output = 0;


    private double currentEncoderTicks = 0;

    private boolean hasVisionTarget = false;
    private Follower follower;

    private Pose testingPose = new Pose (84,85, Math.toRadians(90));

    private PIDController turretPID = new PIDController(0,0,0);

    private PIDController limelightPID = new PIDController(0, 0, 0);
    private double visionOutput = 0;
    private Limelight limelight;


    public void initialize() {

        dash = FtcDashboard.getInstance();

        limelight = new Limelight(hardwareMap);

        limelight.switchPipelineRed();




        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders on Init so we know where 0 is (Assuming Front Facing)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(testingPose);
        follower.startTeleopDrive();
        follower.update();

        register(limelight);







    }

    public void run() {

        LLResult result = limelight.getRawResult();

        currentEncoderTicks = turret.getCurrentPosition();


        tx = (result != null) ? result.getTx() : 0;

        hasVisionTarget = result != null;








        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();


//        if (gamepad1.dpad_right) turret.setPower(0.4);
//        else if (gamepad1.dpad_left) turret.setPower(-0.4);
//         else turret.setPower(0);



        goalDisplacementX = Math.abs((RobotConstraints.redGoal.getX()) - follower.getPose().getX());
        goalDisplacementY = Math.abs((RobotConstraints.redGoal.getY()) - follower.getPose().getY());

        angle = (((((currentEncoderTicks/(double)(COUNTS_PER_REVOLUTION)) * 360))));


        position = Math.toDegrees(Math.atan2(goalDisplacementY, goalDisplacementX)) - Math.toDegrees(follower.getPose().getHeading());


        turretPID.setPID(kP, kI, kD);
        limelightPID.setPID(viskP, viskI, viskD);

        if (angle > rightAngleLimit) {
            angle += 360;


//            if (angle < leftAngleLimit) {
//
//                angle = leftAngleLimit;
//            }
        }

        if (angle < leftAngleLimit) {
            angle -= 360;


//            if (angle > rightAngleLimit) {
//
//                angle = rightAngleLimit;
//            }
        }



        if (hasVisionTarget) {
            // Optional: low-pass filter the tx to smooth jitter
            // Limelight PID: positive tx means target is right, so turret should move right
            visionOutput = -limelightPID.calculate(0, tx); // target=0, measurement=tx
        } else visionOutput = 0.0;


        output = turretPID.calculate(angle, Range.clip(-position, leftAngleLimit, rightAngleLimit));



        turret.setPower(output);





        telemetry.addData("Encoder Ticks", turret.getCurrentPosition());
        telemetry.addData("Encoder Angle", angle);
        telemetry.addData("Angle to rotate to goalington", position);
        telemetry.addData("pose", follower.getPose());
        telemetry.update();


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("PID kP", kP);
        packet.put("PID kI", kI);
        packet.put("PID kD", kD);
        packet.put("Current Angle", angle);
        packet.put("Target Position", -position);
        dash.sendTelemetryPacket(packet);




    }



    public double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }











}




