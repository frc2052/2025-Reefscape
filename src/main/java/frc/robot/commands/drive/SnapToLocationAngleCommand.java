// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SnapToLocationAngleCommand extends SnapToAngleCommand {
  SnapLocation goalSnap;

  public SnapToLocationAngleCommand(
      SnapLocation snapLocations,
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

  public static double getLocationAngleRadians(SnapLocation snapLocations) {
    double angleRadians = 0;
    switch (snapLocations) {
      case FORWARD:
        angleRadians = SnapLocation.FORWARD.getRobotAngle().in(Radians);
      case LeftCoralStation:
        angleRadians = SnapLocation.LeftCoralStation.getRobotAngle().in(Radians);
        break;
      case RightCoralStation:
        angleRadians = SnapLocation.RightCoralStation.getRobotAngle().in(Radians);
        break;
      case ReefAB:
        angleRadians = SnapLocation.ReefAB.getRobotAngle().in(Radians);
        break;
      case ReefCD:
        angleRadians = SnapLocation.ReefCD.getRobotAngle().in(Radians);
        break;
      case ReefEF:
        angleRadians = SnapLocation.ReefEF.getRobotAngle().in(Radians);
        break;
      case ReefGH:
        angleRadians = SnapLocation.ReefGH.getRobotAngle().in(Radians);
        break;
      case ReefIJ:
        angleRadians = SnapLocation.ReefIJ.getRobotAngle().in(Radians);
        break;
      case ReefKL:
        angleRadians = SnapLocation.ReefKL.getRobotAngle().in(Radians);
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

  public enum SnapLocation {
    FORWARD(Degrees.of(0)),
    LeftCoralStation(Degrees.of(306)),
    RightCoralStation(Degrees.of(-306)),
    ReefAB(Degrees.of(180)),
    ReefCD(Degrees.of(240)),
    ReefEF(Degrees.of(300)),
    ReefGH(Degrees.of(0)),
    ReefIJ(Degrees.of(90)),
    ReefKL(Degrees.of(120));

    private Angle robotAngle;

    private SnapLocation(Angle robotAngle) {
      this.robotAngle = robotAngle;
    }

    public Angle getRobotAngle() {
      return robotAngle;
    }
  }
}
