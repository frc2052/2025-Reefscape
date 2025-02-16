// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetFieldLocation;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToLocationAngleCommand extends SnapToAngleCommand {
  TargetFieldLocation goalSnap;

  public SnapToLocationAngleCommand(
      TargetFieldLocation snapLocation,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentricSupplier) {
    super(
        new Rotation2d(snapLocation.getLineupAngle().in(Radians)),
        xSupplier,
        ySupplier,
        rotationSupplier,
        fieldCentricSupplier);

    goalSnap = snapLocation;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentRotation = RobotState.getInstance().getFieldToRobot().getRotation().getDegrees();
    if (currentRotation + 5 > Units.radiansToDegrees(goalSnap.getLineupAngle().in(Radians))
        && currentRotation - 5 < Units.radiansToDegrees(goalSnap.getLineupAngle().in(Radians))) {
      return true;
    } else {
      return false;
    }
  }
}
