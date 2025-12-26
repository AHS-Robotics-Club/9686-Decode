package org.firstinspires.ftc.teamcode.commands;//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.pedropathing.util.NanoTimer;
//
//import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
//import org.firstinspires.ftc.teamcode.subsystems.Kicker;
//
//import java.util.concurrent.TimeUnit;
//
//public class TimedKickCommand extends CommandBase {
//
//    // The subsystem the command runs on
//    private final Kicker kicker;
//
//    private NanoTimer kickTimer, downTimer;
//
//    public TimedKickCommand(Kicker subsystem) {
//        kicker = subsystem;
//        addRequirements(kicker);
//
//        kickTimer = new NanoTimer();
//        downTimer = new NanoTimer();
//    }
//
//    @Override
//    public void initialize() {
//        kicker.kick();
//        kickTimer.resetTimer();
//    }
//
//    @Override
//    public boolean isFinished() {
//
//        if (kickTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.KICKER_KICK_TIME){
//
//            kickTimer.resetTimer();
//
//
//            kicker.down();
//            downTimer.resetTimer();
//
//
//
//            if (downTimer.getElapsedTime(TimeUnit.MILLISECONDS) >= RobotConstraints.KICKER_DOWN_TIME) {
//                return true;
//            }
//
//
//        }
//
//
//        return false;
//    }
//
//}


import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.constants.RobotConstraints;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;

import java.util.concurrent.TimeUnit;

public class TimedKickCommand extends CommandBase {

    private final Kicker kicker;
    private final NanoTimer kickTimer = new NanoTimer();

    private final NanoTimer downTimer = new NanoTimer();


    private enum State {
        KICKING,
        LOWERING
    }

    private State state;

    private boolean hasKicked = false;

    public TimedKickCommand(Kicker subsystem) {
        kicker = subsystem;
        addRequirements(kicker);
    }

//    @Override
//    public void initialize() {
//        kicker.kick();
//        kickTimer.resetTimer();
//        state = State.KICKING;
//    }
//
//    @Override
//    public void execute() {
//        switch (state) {
//            case KICKING:
//                if (kickTimer.getElapsedTime(TimeUnit.MILLISECONDS)
//                        >= RobotConstraints.KICKER_KICK_TIME) {
//                    hasKicked = true;
//                    kicker.down();
//                    downTimer.resetTimer();
//                    state = State.LOWERING;
//                }
//                break;
//
//            case LOWERING:
//                // nothing to do, just wait
//                break;
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return state == State.LOWERING &&
//                downTimer.getElapsedTime(TimeUnit.MILLISECONDS)
//                        >= RobotConstraints.KICKER_DOWN_TIME && hasKicked;
//    }
//}

    @Override
    public void initialize() {
        state = State.KICKING;
        kickTimer.resetTimer();
        hasKicked = false;
    }

    @Override
    public void execute() {
        switch (state) {

            case KICKING:
                kicker.kick();  // ← APPLY EVERY LOOP

                if (kickTimer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.KICKER_KICK_TIME) {

                    kicker.down();
                    downTimer.resetTimer();
                    state = State.LOWERING;
                    hasKicked = true;
                }
                break;

            case LOWERING:
                kicker.down();  // ← HOLD DOWN EVERY LOOP
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.LOWERING &&
                downTimer.getElapsedTime(TimeUnit.MILLISECONDS)
                        >= RobotConstraints.KICKER_DOWN_TIME && hasKicked;
    }
}
