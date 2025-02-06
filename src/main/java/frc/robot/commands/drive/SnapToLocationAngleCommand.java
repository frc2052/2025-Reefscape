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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
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

  public enum SnapLocation { // blue
    FORWARD(Degrees.of(0)),
    LeftCoralStation(Degrees.of(306)),
    RightCoralStation(Degrees.of(-306)),
    ReefAB(Degrees.of(0)), // reverse gh
    ReefCD(Degrees.of(90)), // reverse ij
    ReefEF(Degrees.of(120)), // reverse kl
    ReefGH(Degrees.of(180)),
    ReefIJ(Degrees.of(240)),
    ReefKL(Degrees.of(300));

    private Angle robotAngle;

    private SnapLocation(Angle robotAngle) {
      this.robotAngle = robotAngle;
    }

    public Angle getRobotAngle() {
      return robotAngle;
    }
  }

  public static SnapLocation getSnapLocationByID(int id) {
    SnapLocation location;
    switch (id) {
      case 18:
        location = SnapLocation.ReefAB;
        break;
      case 17:
        location = SnapLocation.ReefCD;
        break;
      case 22:
        location = SnapLocation.ReefEF;
        break;
      case 21:
        location = SnapLocation.ReefGH;
        break;
      case 20:
        location = SnapLocation.ReefIJ;
        break;
      case 19:
        location = SnapLocation.ReefKL;
        break;
      case 10:
        location = SnapLocation.ReefGH;
        break;
      case 11:
        location = SnapLocation.ReefIJ;
        break;
      case 6:
        location = SnapLocation.ReefKL;
        break;
      case 7:
        location = SnapLocation.ReefAB;
        break;
      case 8:
        location = SnapLocation.ReefCD;
        break;
      case 9:
        location = SnapLocation.ReefEF;
        break;
      default:
        location = SnapLocation.FORWARD;
    }

    return location;
  }

  public static double getAlignAngleByID(int id) {
    return getLocationAngleRadians(getSnapLocationByID(id));
  }
}
