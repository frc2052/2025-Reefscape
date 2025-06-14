// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DrivetrainConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SnapToAngleCommand extends DefaultDriveCommand {
    private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(super.maxSpeed * 0.05)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    private Rotation2d desiredDirection;

    public SnapToAngleCommand(
            Rotation2d desiredDirection,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldCentricSupplier) {
        super(xSupplier, ySupplier, rotationSupplier, fieldCentricSupplier);
        this.desiredDirection = desiredDirection;

        drive.HeadingController.setPID(3.5, 0, 0);
        drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        drive.HeadingController.setTolerance(DrivetrainConstants.HEADING_TOLERANCE.in(Radians));
    }

    @Override
    public SwerveRequest getSwerveRequest() {
        drive.withTargetDirection(desiredDirection)
                .withVelocityX(getX() * super.maxSpeed)
                .withVelocityY(getY() * super.maxSpeed);

        return drive;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
