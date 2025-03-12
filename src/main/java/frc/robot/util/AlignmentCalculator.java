package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;

public class AlignmentCalculator {
    // public static Pose2d scaleFromFieldElement(
    //     Pose2d initial, Translation2d offset, Translation2d fieldElementTranslation) {
    //   Translation2d fieldElementLoc = fieldElementTranslation;

    //   Translation2d relativeTranslation = initial.getTranslation().minus(fieldElementLoc);
    //   double radius = relativeTranslation.getNorm();
    //   Rotation2d theta =
    //       new Rotation2d(-Math.atan(relativeTranslation.getY() / relativeTranslation.getX()));
    //   double newRadius = radius + offset.getMeasureX().in(Meters);

    //   Rotation2d rotateValue = Rotation2d.fromDegrees(180); // front of robot pointed @ target:
    // reef

    //   Pose2d pose =
    //       new Pose2d(
    //           new Translation2d(
    //                   Math.copySign(
    //                       newRadius * Math.cos(theta.getRadians()), relativeTranslation.getX()),
    //                   Math.copySign(
    //                       newRadius * Math.sin(theta.getRadians()), relativeTranslation.getY()))
    //               .plus(fieldElementLoc),
    //           initial.getRotation().rotateBy(rotateValue));

    //   Translation2d horizontalOffset = new Translation2d(0, offset.getMeasureY().in(Meters));
    //   Translation2d relativePose = horizontalOffset.rotateBy(pose.getRotation());

    //   return new Pose2d(relativePose.plus(pose.getTranslation()), pose.getRotation());
    // }

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
    // public static Rotation2d getRotationToReef(Translation2d robotTranslation, boolean isRedReef) {
    //   Translation2d reefLocation;
    //   if (isRedReef) {
    //     reefLocation = FieldConstants.RED_REEF_CENTER;
    //   } else {
    //     reefLocation = FieldConstants.BLUE_REEF_CENTER;
    //   }
    //   // Calculate the differences in x and y coordinates
    //   double deltaX = reefLocation.getX() - robotTranslation.getX();
    //   double deltaY = reefLocation.getY() - robotTranslation.getY();

    //   // Use Math.atan2 to calculate the angle in radians
    //   return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX));
    // }

    // public static Pose2dPolar getPositionFromReef(Pose2d pose, boolean isRedReef) {
    //   Translation2d reefLocation;
    //   if (isRedReef) {
    //     reefLocation = FieldConstants.RED_REEF_CENTER;
    //   } else {
    //     reefLocation = FieldConstants.BLUE_REEF_CENTER;
    //   }

    //   Polar2d polar = new Polar2d(pose.getTranslation().minus(reefLocation));

    //   return new Pose2dPolar(pose.getRotation(), polar);
    // }
    public enum FieldElementFace {
        AB(7, 18, Degrees.of(0)),
        CD(8, 17, Degrees.of(60)),
        EF(9, 22, Degrees.of(120)),

        GH(10, 21, Degrees.of(180)),
        IJ(11, 20, Degrees.of(240)),
        KL(6, 19, Degrees.of(300)),
        LCS(1, 13, Degrees.of(306)),
        RCS(2, 12, Degrees.of(-306));

        public final int redTagID;
        public final int blueTagID;
        public final Angle lineupAngle;

        private FieldElementFace(int redTagID, int blueTagID, Angle lineupAngle) {
            this.redTagID = redTagID;
            this.blueTagID = blueTagID;
            this.lineupAngle = lineupAngle;
        }

        public int getTagID() {
            return RobotState.getInstance().isRedAlliance() ? redTagID : blueTagID;
        }

        public Angle getLineupAngle() {
            return lineupAngle;
        }
    }

    public enum AlignOffset {
        LEFT_BRANCH,
        MIDDLE_REEF,
        RIGHT_BRANCH;
    }
}
