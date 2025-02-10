package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import com.team2052.lib.geometry.Polar2d;
import com.team2052.lib.geometry.Pose2dPolar;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.AlignWithFieldElementCommand.FieldElement;

import org.littletonrobotics.junction.Logger;

public class AimingCalculator {

  public static Pose2d scaleAll(Pose2d initial, Distance add, FieldElement fieldElem){
    Translation2d fieldElementLoc = fieldElem.getFieldPosition();

    Logger.recordOutput(fieldElem.getDisplayName() + " location", fieldElementLoc);

    Translation2d relativeTranslation = initial.getTranslation().minus(fieldElementLoc);
    double radius = relativeTranslation.getNorm();
    Rotation2d theta =
        new Rotation2d(-Math.atan(relativeTranslation.getY() / relativeTranslation.getX()));
    double newRadius = radius + add.in(Meters);

    Rotation2d rotateValue = Rotation2d.fromDegrees(180); // front of robot pointed @ target: reef

    if(FieldElement.checkIsProcessor(fieldElem)){
      rotateValue = Rotation2d.fromDegrees(90); // ground pickup / processor score side pointed @ target?
    }

    return new Pose2d(
      new Translation2d(
            Math.copySign(newRadius * Math.cos(theta.getRadians()), relativeTranslation.getX()),
            Math.copySign(newRadius * Math.sin(theta.getRadians()), relativeTranslation.getY()))
        .plus(fieldElementLoc),
      initial.getRotation().rotateBy(rotateValue));
  }


  public static Pose2d scaleFromReef( // adjusts for scoring location
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

  public enum ElementFieldPosition{
    BLUE_REEF(new Translation2d(Inches.of(177.06927), Inches.of(158.5))),
    RED_REEF(new Translation2d(Inches.of(177.06927), Inches.of(158.5))),

    BLUE_PROCESSOR(new Translation2d()),
    RED_PROCESSOR(new Translation2d()),

    REDSIDE_LEFT_STATION(new Translation2d()),
    REDSIDE_RIGHT_STAITON(new Translation2d()),

    BLUESIDE_LEFT_STATION(new Translation2d()),
    BLUESIDE_RIGHT_STATION(new Translation2d());

    private Translation2d elemPose;

    public Translation2d getElemPose(){
      return elemPose;
    }

    private ElementFieldPosition(Translation2d translation){
      elemPose = translation;
    }
  }
}
