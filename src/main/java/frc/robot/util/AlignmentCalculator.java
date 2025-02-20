package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.team2052.lib.geometry.Polar2d;
import com.team2052.lib.geometry.Pose2dPolar;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;
import java.util.Optional;

public class AlignmentCalculator {

  public static Pose2d scaleFromFieldElement(
      Pose2d initial, Translation2d offset, Translation2d fieldElementTranslation) {
    Translation2d fieldElementLoc = fieldElementTranslation;

    Translation2d relativeTranslation = initial.getTranslation().minus(fieldElementLoc);
    double radius = relativeTranslation.getNorm();
    Rotation2d theta =
        new Rotation2d(-Math.atan(relativeTranslation.getY() / relativeTranslation.getX()));
    double newRadius = radius + offset.getMeasureX().in(Meters);

    Rotation2d rotateValue = Rotation2d.fromDegrees(180); // front of robot pointed @ target: reef

    Pose2d pose =
        new Pose2d(
            new Translation2d(
                    Math.copySign(
                        newRadius * Math.cos(theta.getRadians()), relativeTranslation.getX()),
                    Math.copySign(
                        newRadius * Math.sin(theta.getRadians()), relativeTranslation.getY()))
                .plus(fieldElementLoc),
            initial.getRotation().rotateBy(rotateValue));

    Translation2d horizontalOffset = new Translation2d(0, offset.getMeasureY().in(Meters));
    Translation2d relativePose = horizontalOffset.rotateBy(pose.getRotation());

    return new Pose2d(relativePose.plus(pose.getTranslation()), pose.getRotation());
  }

  // public static Pose2d scaleFromReef( // adjusts for scoring location
  //     Pose2d initialPose, Distance addedDistance, boolean isRedReef) {
  //   Translation2d reefLocation;
  //   if (isRedReef) {
  //     reefLocation = FieldConstants.RED_REEF_CENTER;
  //   } else {
  //     reefLocation = FieldConstants.BLUE_REEF_CENTER;
  //   }

  //   Logger.recordOutput("reef location", reefLocation);

  //   Translation2d relativeTranslation = initialPose.getTranslation().minus(reefLocation);
  //   double radius = relativeTranslation.getNorm();
  //   Rotation2d theta =
  //       new Rotation2d(-Math.atan(relativeTranslation.getY() / relativeTranslation.getX()));
  //   double newRadius = radius + addedDistance.in(Meters);

  //   return new Pose2d(
  //       new Translation2d(
  //               Math.copySign(newRadius * Math.cos(theta.getRadians()),
  // relativeTranslation.getX()),
  //               Math.copySign(newRadius * Math.sin(theta.getRadians()),
  // relativeTranslation.getY()))
  //           .plus(reefLocation),
  //       initialPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  // }

  // public static Pose2d horizontalAjustment(Distance ajustment, Pose2d pose) {
  //   Translation2d offset = new Translation2d(0, ajustment.in(Meters));
  //   Translation2d relativePose = offset.rotateBy(pose.getRotation());

  //   return new Pose2d(relativePose.plus(pose.getTranslation()), pose.getRotation());
  // }
  public static Rotation2d getRotationToReef(Translation2d robotTranslation, boolean isRedReef) {
    Translation2d reefLocation;
    if (isRedReef) {
      reefLocation = FieldConstants.RED_REEF_CENTER;
    } else {
      reefLocation = FieldConstants.BLUE_REEF_CENTER;
    }
    // Calculate the differences in x and y coordinates
    double deltaX = reefLocation.getX() - robotTranslation.getX();
    double deltaY = reefLocation.getY() - robotTranslation.getY();

    // Use Math.atan2 to calculate the angle in radians
    return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX));
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

  public enum TargetFieldLocation {
    AB(
        7,
        18,
        Degrees.of(0),
        FieldConstants.RED_REEF_CENTER,
        FieldConstants.BLUE_REEF_CENTER,
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    CD(
        8,
        17,
        Degrees.of(60),
        FieldConstants.RED_REEF_CENTER,
        FieldConstants.BLUE_REEF_CENTER,
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    EF(
        9,
        22,
        Degrees.of(120),
        FieldConstants.RED_REEF_CENTER,
        FieldConstants.BLUE_REEF_CENTER,
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    GH(
        10,
        21,
        Degrees.of(180),
        FieldConstants.RED_REEF_CENTER,
        FieldConstants.BLUE_REEF_CENTER,
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    IJ(
        11,
        20,
        Degrees.of(240),
        FieldConstants.RED_REEF_CENTER,
        FieldConstants.BLUE_REEF_CENTER,
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    KL(
        6,
        19,
        Degrees.of(300),
        FieldConstants.RED_REEF_CENTER,
        FieldConstants.BLUE_REEF_CENTER,
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    RCS(
        2,
        12,
        Degrees.of(-306),
        new Translation2d(),
        new Translation2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    LCS(
        1,
        13,
        Degrees.of(306),
        new Translation2d(),
        new Translation2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()),
    PRC(
        3,
        16,
        Degrees.of(0),
        new Translation2d(),
        new Translation2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d(),
        new Transform2d()); // Processor (side of the field, not where our alliance's human player
    // is)

    public final int redTagID;
    public final int blueTagID;
    public final Angle lineupAngle;
    private final Translation2d redElementTranslation;
    private final Translation2d blueElementTranslation;
    private final Transform2d leftBranchNudgeBlue;
    private final Transform2d rightBranchNudgeBlue;
    private final Transform2d leftBranchNudgeRed;
    private final Transform2d rightBranchNudgeRed;

    private TargetFieldLocation(
        int redTagID,
        int blueTagID,
        Angle lineupAngle,
        Translation2d redElementTranslation,
        Translation2d blueElementTranslation,
        Transform2d leftBranchNudgeBlue,
        Transform2d rightBranchNudgeBlue,
        Transform2d leftBranchNudgeRed,
        Transform2d rightBranchNudgeRed) {
      this.redTagID = redTagID;
      this.blueTagID = blueTagID;
      this.lineupAngle = lineupAngle;
      this.redElementTranslation = redElementTranslation;
      this.blueElementTranslation = blueElementTranslation;
      this.leftBranchNudgeBlue = leftBranchNudgeBlue;
      this.rightBranchNudgeBlue = rightBranchNudgeBlue;
      this.leftBranchNudgeRed = leftBranchNudgeRed;
      this.rightBranchNudgeRed = rightBranchNudgeRed;
    }

    public int getTagID() {
      return RobotState.getInstance().isRedAlliance() ? redTagID : blueTagID;
    }

    public Angle getLineupAngle() {
      return lineupAngle;
    }

    public Transform2d getLeftBranchNudge() {
      return RobotState.getInstance().isRedAlliance() ? leftBranchNudgeRed : leftBranchNudgeBlue;
    }

    public Transform2d getRightBranchNudge() {
      return RobotState.getInstance().isRedAlliance() ? rightBranchNudgeRed : rightBranchNudgeBlue;
    }

    public Translation2d getElementTranslation() {
      return RobotState.getInstance().isRedAlliance()
          ? redElementTranslation
          : blueElementTranslation;
    }

    public Pose2d getWithOffset(AlignOffset offset) {
      return getWithTransform(offset.getTransform());
    }

    public Pose2d getWithTransform(Transform2d transform) {
      Translation2d elementTranslation = getElementTranslation();
      Optional<Pose3d> tagPose =
          FieldConstants.DEFAULT_APRIL_TAG_LAYOUT_TYPE.layout.getTagPose(getTagID());
      if (tagPose.isPresent()) {
        return scaleFromFieldElement(
            tagPose.get().toPose2d(), transform.getTranslation(), elementTranslation);
      } else {
        return new Pose2d(elementTranslation, Rotation2d.fromDegrees(0));
      }
    }

    public boolean getIsReef() {
      return this == AB || this == CD || this == EF || this == GH || this == IJ || this == KL;
    }
  }

  /* Align Offset is relative to the tag poses on the field element */
  public enum AlignOffset {
    LEFT_REEF_LOC(new Transform2d(Meters.of(0.47), Inches.of(7), new Rotation2d(Math.PI))),
    MIDDLE_REEF_LOC(new Transform2d(0.55, 0.0, new Rotation2d(Math.PI))),
    RIGHT_REEF_LOC(new Transform2d(Meters.of(0.47), Inches.of(-7), new Rotation2d(Math.PI))),

    ALGAE_REEF_LOC(new Transform2d(0.5, 0.0, new Rotation2d(Math.PI / 2))),

    PROCESSOR_MIDDLE_LOC(new Transform2d(0.5, -0.25, new Rotation2d(Math.PI))),

    LEFT_CORAL_STATION_LOC(new Transform2d(0.5, -0.25, new Rotation2d(Math.PI))),
    RIGHT_CORAL_STATION_LOC(new Transform2d(0.5, -0.25, new Rotation2d(Math.PI)));

    public final Transform2d transform;

    private AlignOffset(Transform2d gt) {
      this.transform = gt;
    }

    public Transform2d getTransform() {
      return transform;
    }
  }
}
