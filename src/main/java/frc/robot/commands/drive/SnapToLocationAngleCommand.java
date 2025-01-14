// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToLocationAngleCommand extends SnapToAngleCommand {
  SnapLocations goalSnap;
  /** Creates a new SnapToLocationAngleCommand. */
  public SnapToLocationAngleCommand(
      SnapLocations snapLocations,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentricSupplier) {
    super(
        new Rotation2d(getLocationAngleRadians(snapLocations)),
        xSupplier,
        ySupplier,
        rotationSupplier,
        fieldCentricSupplier);

    goalSnap = snapLocations;
  }

  public static double getLocationAngleRadians(SnapLocations snapLocations) {
    double angleRadians = 0;
    switch (snapLocations) {
      case FORWARD:
        angleRadians = 0;
      case LeftCoralStation:
        angleRadians = Constants.FieldAndRobotConstants.LEFT_CORAL_STATION_ANGLE_RADIANS;
        break;
      case RightCoralStation:
        angleRadians = Constants.FieldAndRobotConstants.RIGHT_CORAL_STATION_ANGLE_RADIANS;
        break;
      case ReefAB:
        angleRadians = Constants.FieldAndRobotConstants.REEF_AB;
        break;
      case ReefCD:
        angleRadians = Constants.FieldAndRobotConstants.REEF_CD;
        break;
      case ReefEF:
        angleRadians = Constants.FieldAndRobotConstants.REEF_EF;
        break;
      case ReefGH:
        angleRadians = Constants.FieldAndRobotConstants.REEF_GH;
        break;
      case ReefIJ:
        angleRadians = Constants.FieldAndRobotConstants.REEF_IJ;
        break;
      case ReefKL:
        angleRadians = Constants.FieldAndRobotConstants.REEF_KL;
        break;
    }

    return angleRadians;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentRotation = RobotState.getInstance().getFieldToRobot().getRotation().getDegrees();
    if (currentRotation + 5 > Units.radiansToDegrees(getLocationAngleRadians(goalSnap))
        && currentRotation - 5 < Units.radiansToDegrees(getLocationAngleRadians(goalSnap))) {
      return true;
    } else {
      return false;
    }
  }

  public enum SnapLocations {
    FORWARD,
    LeftCoralStation,
    RightCoralStation,
    ReefAB,
    ReefCD,
    ReefEF,
    ReefGH,
    ReefIJ,
    ReefKL
  }
}
