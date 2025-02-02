package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import com.team2052.lib.geometry.Polar2d;
import com.team2052.lib.geometry.Pose2dPolar;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AimingCalculator {

  public static Pose2d scaleFromReef(
      Pose2d initialPose, Distance addedDistance, boolean isRedReef) {
    Translation2d reefLocation;
    if (isRedReef) {
      reefLocation = FieldConstants.RED_REEF_CENTER;
    } else {
      reefLocation = FieldConstants.BLUE_REEF_CENTER;
    }
    Logger.recordOutput("reef location", reefLocation);

    Translation2d relativeTranslation = initialPose.getTranslation().minus(reefLocation);

    double radius = relativeTranslation.getNorm();

    Rotation2d theta =
        new Rotation2d(-Math.atan(relativeTranslation.getY() / relativeTranslation.getX()));

    double newRadius = radius + addedDistance.in(Meters);

    return new Pose2d(
        new Translation2d(
                Math.copySign(newRadius * Math.cos(theta.getRadians()), relativeTranslation.getX()),
                Math.copySign(newRadius * Math.sin(theta.getRadians()), relativeTranslation.getY()))
            .plus(reefLocation),
        initialPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  public static Pose2d horizontalAjustment(Distance ajustment, Pose2d pose) {
    Translation2d offset = new Translation2d(0, ajustment.in(Meters));
    Translation2d relativePose = offset.rotateBy(pose.getRotation());

    return new Pose2d(relativePose.plus(pose.getTranslation()), pose.getRotation());
  }

  public static Pose2dPolar getPositionFromReef(Pose2d pose, boolean isRedReef) {
    Translation2d reefLocation;
    if (isRedReef) {
      reefLocation = FieldConstants.RED_REEF_CENTER;
    } else {
      reefLocation = FieldConstants.BLUE_REEF_CENTER;
    }

    Polar2d polar = new Polar2d(pose.getTranslation().minus(reefLocation));

    return new Pose2dPolar(pose.getRotation(), polar);
  }
}
