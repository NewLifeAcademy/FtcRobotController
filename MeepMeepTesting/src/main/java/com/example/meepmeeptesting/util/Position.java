package com.example.meepmeeptesting.util;

import com.acmerobotics.roadrunner.Pose2d;

public class Position {
    Pose2d pose;
    double tangent;

    public Position(Pose2d pose, double tangent) {
        this.pose = pose;
        this.tangent = tangent;
    }

    public Pose2d getPose() {
        return pose;
    }
    public double getTangent() {
        return tangent;
    }
}