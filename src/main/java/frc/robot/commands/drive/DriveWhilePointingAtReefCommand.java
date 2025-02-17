// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.util.AlignmentCalculator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveWhilePointingAtReefCommand extends DefaultDriveCommand {
  Supplier<Rotation2d> targetHeadingSupplier =
      () ->
          AlignmentCalculator.getRotationToReef(
              RobotState.getInstance().getFieldToRobot().getTranslation(),
              RobotState.getInstance().isRedAlliance());

  public DriveWhilePointingAtReefCommand(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier fieldCentricSupplier) {
    super(xSupplier, ySupplier, () -> 0, fieldCentricSupplier);

    driveWithHeading.HeadingController.setPID(3.5, 0.0, 0.0);
    driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    return driveWithHeading
        .withVelocityX(getX() * maxSpeed)
        .withVelocityY(getY() * maxSpeed)
        .withTargetDirection(targetHeadingSupplier.get());
  }
}
