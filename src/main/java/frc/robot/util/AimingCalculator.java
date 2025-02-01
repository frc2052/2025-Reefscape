package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;

public class AimingCalculator {

  public static Pose2d scaleFromReef(
      Pose2d initialPose, Distance addedDistance, boolean isRedAlliance) {
    Translation2d reefLocation;
    if (isRedAlliance) {
      reefLocation = FieldConstants.RED_REEF_CENTER;
    } else {
      reefLocation = FieldConstants.BLUE_REEF_CENTER;
    }

    Translation2d relativeTranslation = reefLocation.plus(initialPose.getTranslation());

    double radius = relativeTranslation.getNorm();
    Rotation2d theta =
        new Rotation2d(Math.atan2(relativeTranslation.getX(), relativeTranslation.getY()));

    double newRadius = radius + addedDistance.in(Meters);

    return new Pose2d(
        new Translation2d(newRadius, theta).minus(reefLocation), initialPose.getRotation());
  }
}
