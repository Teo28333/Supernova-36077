package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.opmode.Auto.Path.AutoPath;


public class RobotAuton {

    private final Follower follower;
    public RobotAuton(Follower follower) {
        this.follower = follower;
    }


        public void start(Pose startingPose) {
            follower.setPose(startingPose);
        }


        public void followPath(Path path) {
            follower.follow(path);
        }


        public void update() {
            follower.update();
        }


        public boolean isPathFinished() {
            return follower.atParametricEnd();
        }


        public Follower getFollower() {
            return follower;
        }


        public Pose getPose() {
            return follower.pose();
        }

    public boolean isBusy() {
        return follower.isBusy();
    }
    }



