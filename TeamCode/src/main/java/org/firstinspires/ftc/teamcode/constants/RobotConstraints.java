package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class RobotConstraints {

    public static Pose redGoal = new Pose(140,140);

    public static Pose blueGoal = new Pose(8,139);

    public static Pose motifPose = new Pose(72,142);

    public static Pose teleOpStartPose = new Pose(121, 125, Math.toRadians(38));

    public static long

            //these are in millis, used for timing capability FIND!

    SPINDEX_120_DEG_ROT_TIME = 250, //550

    SPINDEX_120_DEG_COLOR_STAB_TIME = 300,
    SPINDEX_60_DEG_ROT_TIME = 250, //300

    INTAKE_CD_READING_TIME = 35,
            OUTTAKE_CD_READING_TIME = 75,
    KICKER_KICK_TIME = 95, //200
    KICKER_DOWN_TIME = 75,

    //200



    //these are in MM, to be found experimentally later

    INTAKE_BALL_CHAMBERED_DISTANCE = 57,

    INTAKE_POSITION_NO_BALL_THRESH = 60,

    OUTTAKE_BALL_POSITION_THRESH = 70;




    public static double

            OUTTAKE_GREEN_BALL_THRESH = 0.0185,
            OUTTAKE_GREEN_BALL_HUE_THRESH = 170,

            INTAKE_GREEN_BALL_THRESH = 0.0155;


    ;






}
