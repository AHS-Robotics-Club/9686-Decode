package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private final Path path;
    private final boolean isPathChain;
    private final boolean holdEnd;

    public FollowPathCommand(Follower follower, PathChain pathChain) {
        this(follower, pathChain, false);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.path = null;
        this.isPathChain = true;
        this.holdEnd = holdEnd;
    }

    public FollowPathCommand(Follower follower, Path path) {
        this(follower, path, false);
    }

    public FollowPathCommand(Follower follower, Path path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.pathChain = null;
        this.isPathChain = false;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        if (isPathChain) {
            follower.followPath(pathChain, holdEnd);
        } else {
            // Assuming followPath(Path, boolean) exists, if not we might need to verify or
            // wrap Path in PathChain
            follower.followPath(path, holdEnd);
        }
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
