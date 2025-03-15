package com.team2052.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pose2dPolar {
    private final Rotation2d rotation;
    private final Polar2d polar;

    public Pose2dPolar(Rotation2d rotation, Polar2d polarPose) {
        this.rotation = rotation;
        this.polar = polarPose;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public Polar2d getPolarPose() {
        return polar;
    }

    public Pose2d toPose2d() {
        return new Pose2d(polar.toTranslation2d(), rotation);
    }
}
