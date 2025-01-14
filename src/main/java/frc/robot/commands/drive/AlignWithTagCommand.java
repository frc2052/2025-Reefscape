// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocations;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignWithTagCommand extends DefaultDriveCommand {
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final PIDController yController;
  private boolean lockedOnTag = false;
  private PhotonTrackedTarget target;

  private SwerveRequest.FieldCentricFacingAngle drive =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(super.maxSpeed * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithTagCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) { // add enum supplier for scoring position, left middle or right
    super(xSupplier, ySupplier, rotationSupplier, () -> false);
    yController = new PIDController(1.75, 0, 0);
    yController.setTolerance(0.5);
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if(!lockedOnTag) {
      return super.getSwerveRequest();
    } else {
      return drive.withTargetDirection(getDirection());
    }
  }

  @Override
  public double getY() {
    if (!lockedOnTag) {
      return super.getY();
    } else {
      // put stuff here
      return 0;
    }
  }

  public double getDirection() {
    if (!lockedOnTag) {
      return super.getRotation();
    } else {
      switch(target.fiducialId) {
        case 18:
           SnapLocations.ReefAB
      }

      return 0;
    }
  }

  @Override
  public boolean getFieldCentric() {
    if (!lockedOnTag) {
      return super.getFieldCentric();
    } else {
      return false;
    }
  }

  @Override
  public void execute() {
    lockedOnTag = vision.getReefCamClosestTarget().isPresent();
    if (lockedOnTag) {
      target = vision.getReefCamClosestTarget().get();
    }
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
