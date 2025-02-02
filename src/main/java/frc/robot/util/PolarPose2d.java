package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PolarPose2d {
    private final Rotation2d rotation;
    private final Polar2d polar;

    public PolarPose2d(Rotation2d rotation, Polar2d polarPose) {
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