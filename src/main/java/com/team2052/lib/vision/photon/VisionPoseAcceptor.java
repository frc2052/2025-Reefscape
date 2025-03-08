package com.team2052.lib.vision.photon;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.FieldConstants;

public class VisionPoseAcceptor {

  public static boolean shouldAccept(
      PoseEstimate visionUpdate, double robotVelocity, Pose2d robotPose, boolean isInAuto) {

    if (visionUpdate == null) {
      return false;
    }

    Pose3d estimatedPose = visionUpdate.estimatedPose;

    if (estimatedPose.getTranslation().getX() < -VisionConstants.FIELD_BORDER_MARGIN.in(Meters)
        || estimatedPose.getTranslation().getX()
            > FieldConstants.FIELD_LENGTH.in(Meters)
                + VisionConstants.FIELD_BORDER_MARGIN.in(Meters)
        || estimatedPose.getTranslation().getY() < -VisionConstants.FIELD_BORDER_MARGIN.in(Meters)
        || estimatedPose.getTranslation().getY()
            > FieldConstants.FIELD_WIDTH.in(Meters)
                + VisionConstants.FIELD_BORDER_MARGIN.in(Meters)) {
      return false;
    }

    if (robotVelocity > 4.0) {
      return false;
    }

    if (visionUpdate.highestAmbiguity > VisionConstants.MAX_POSE_AMBIGUITY) {
      return false;
    }

    if (isInAuto) {
      if (estimatedPose.getTranslation().toTranslation2d().getDistance(robotPose.getTranslation())
          > VisionConstants.MAX_VISION_CORRECTION.in(Meters)) {
        return false;
      }
    }

    return true;
  }
}
