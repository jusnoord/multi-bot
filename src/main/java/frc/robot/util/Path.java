// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

/** Add your docs here. */
public class Path {
    private final List<Pose2d> uninterpolatedWaypoints;
    private List<Pose2d> waypoints;
    private double lookAhead;
    private Rotation2d rotationalLookAhead;
    private final double defaultSpeed;
    private int currentWaypointIndex = 0;
    private final double rotationalSpeedCap = 10; // degrees per second
    private boolean isPathComplete = false;
    Supplier<Pose2d> start;

    public Path(Supplier<Pose2d> start, List<Pose2d> waypoints, double defaultSpeed, double lookAhead, Rotation2d rotationalLookAhead) {
        this.defaultSpeed = defaultSpeed;
        this.start = start;
        this.lookAhead = lookAhead;
        this.rotationalLookAhead = rotationalLookAhead;
        // waypoints.add(0, start.get());
        this.uninterpolatedWaypoints = waypoints;
        // this.waypoints = interpolate(waypoints);
    }
    public void init() {
        waypoints = uninterpolatedWaypoints;
        // waypoints.add(0, start.get());
        waypoints = interpolate(waypoints);
        currentWaypointIndex = 0;
        isPathComplete = false;
    }

    public List<Pose2d> interpolate(List<Pose2d> waypoints) {
        // boolean invertRotation = Math.abs(waypoints.get(0).getRotation().minus(waypoints.get(1).getRotation()).getDegrees()) > 90;
        // if (invertRotation) {
        //     for (int i = 1; i < waypoints.size(); i++) {
        //         waypoints.set(i, new Pose2d(waypoints.get(i).getTranslation(), waypoints.get(i).getRotation().plus(Rotation2d.k180deg)));
        //     }
        // }
        List<Pose2d> res = new ArrayList<>();
        double desiredDistance = lookAhead / 4;
        double desiredRotationDistance = rotationalLookAhead.getDegrees() / 4;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Pose2d start = waypoints.get(i);
            Pose2d end = waypoints.get(i + 1);
            double distance = start.getTranslation().getDistance(end.getTranslation());
            double rotationalDistance = start.getRotation().minus(end.getRotation()).getDegrees();
            int numInterpolations = (int)Math.max(distance / desiredDistance, rotationalDistance / desiredRotationDistance);
            for (int j = 0; j < numInterpolations; j++) {
                double t = ((double)j) / (numInterpolations);
                Pose2d interpolated = start.interpolate(end, t);
                res.add(interpolated);
            }
        }
        res.add(waypoints.get(waypoints.size() - 1));
        return res;
    }

    private Pose2d getNextWaypoint(Pose2d currentPose) {
        // iterate through waypoints to find the next one beyond the lookahead distance
        while (currentWaypointIndex < waypoints.size()) {
            Pose2d waypoint = waypoints.get(currentWaypointIndex);
            double distance = currentPose.getTranslation().getDistance(waypoint.getTranslation());
            double rotationalDistance = Math.abs(waypoint.getRotation().minus(currentPose.getRotation()).getRotations());
            rotationalDistance -= 180*(int)((rotationalDistance + 90)/ 180);
            rotationalDistance = Math.abs(rotationalDistance);
            if (distance > lookAhead || rotationalDistance > rotationalLookAhead.getRotations()) {
                Pose2d waypointToReturn = currentWaypointIndex == 0 ? waypoint : waypoints.get(currentWaypointIndex-1);
                double error = (currentPose.getRotation().getDegrees() - waypointToReturn.getRotation().getDegrees());
                int numShortSpins = (int)((error + ((error > 0) ? 90 : -90)) / 180);// amount to add onto target
                return new Pose2d(waypointToReturn.getTranslation(), waypointToReturn.getRotation().plus(Rotation2d.fromDegrees(numShortSpins * 180)));
            }
            currentWaypointIndex++;
        }

        // if no waypoint is found, return the last one
        return waypoints.get(waypoints.size() - 1); 
    }

    public Pose2d getVelocity(Pose2d currentPose, PIDController angleController) {
        Pose2d nextWaypoint = getNextWaypoint(currentPose);
        double distance = currentPose.getTranslation().getDistance(nextWaypoint.getTranslation());
        double speed;
        if (distance >= lookAhead) {
            speed = defaultSpeed; 
        } else {
            speed = defaultSpeed * distance / lookAhead; 
        }
        double dx = nextWaypoint.getX() - currentPose.getX();
        double dy = nextWaypoint.getY() - currentPose.getY();
        double angle = Math.atan2(dy, dx);
        
        double angularSpeed = -angleController.calculate(currentPose.getRotation().getDegrees(), nextWaypoint.getRotation().getDegrees());
        if (Math.abs(angularSpeed) > rotationalSpeedCap) {
            angularSpeed = Math.copySign(rotationalSpeedCap, angularSpeed);
        }

        isPathComplete = currentWaypointIndex >= waypoints.size() - 1 && distance < (lookAhead * 2);

        return new Pose2d(speed * Math.cos(angle), speed * Math.sin(angle), Rotation2d.fromDegrees(angularSpeed));
    }

    public void reset() {
        currentWaypointIndex = 0;
        isPathComplete = false;
    }

    public boolean isFinished() {
        return isPathComplete;
    }
}
