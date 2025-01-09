// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToLocationAngleCommand extends SnapToAngleCommand {
  /** Creates a new SnapToLocationAngleCommand. */
  public SnapToLocationAngleCommand(
    SnapLocations snapLocations,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier rotationSupplier,
    BooleanSupplier fieldCentricSupplier
  ) {
    super(new Rotation2d(getLocationAngleRadians(snapLocations)), xSupplier, ySupplier, rotationSupplier, fieldCentricSupplier);
  }

  private static double getLocationAngleRadians(SnapLocations snapLocations) {
    double angleRadians = 0;
    switch (snapLocations) {
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
    return false;
  }

  public enum SnapLocations {
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
