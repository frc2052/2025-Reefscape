// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocations;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignWithTagCommand extends DefaultDriveCommand {
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private PhotonTrackedTarget target;
  private PIDController yController;
  private Timer targetTimer = new Timer();

  private SwerveRequest.RobotCentric drive =
      new SwerveRequest.RobotCentric()
          .withDeadband(super.maxSpeed * 0.05)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithTagCommand(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier
          rotationSupplier) { // add enum supplier for scoring position, left middle or right
    super(xSupplier, ySupplier, rotationSupplier, () -> false);
    yController = new PIDController(1.5, 0, 0.1);
    yController.setSetpoint(0);
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if (target != null) {
      return drive.withVelocityX(super.getX()).withVelocityY(getYController() * super.maxSpeed);
    } else {
      return super.getSwerveRequest();
    }
  }

  public double getYController() {
    System.out.println("num " + yController.calculate(target.getBestCameraToTarget().getY()));
    return yController.calculate(target.getBestCameraToTarget().getY());
  }

  public SnapLocations getDirection() {
    SnapLocations location;
    switch (target.fiducialId) {
      case 18:
        location = SnapLocations.ReefAB;
        break;
      case 17:
        location = SnapLocations.ReefCD;
        break;
      case 22:
        location = SnapLocations.ReefEF;
        break;
      case 21:
        location = SnapLocations.ReefGH;
        break;
      case 20:
        location = SnapLocations.ReefIJ;
        break;
      case 19:
        location = SnapLocations.ReefKL;
        break;
      case 10:
        location = SnapLocations.ReefGH;
        break;
      case 11:
        location = SnapLocations.ReefIJ;
        break;
      case 6:
        location = SnapLocations.ReefKL;
        break;
      case 7:
        location = SnapLocations.ReefAB;
        break;
      case 8:
        location = SnapLocations.ReefCD;
        break;
      case 9:
        location = SnapLocations.ReefEF;
        break;
      default:
        location = SnapLocations.FORWARD;
    }

    return location;
  }

  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent() && tar.get().getBestCameraToTarget() != null) {
      targetTimer.restart();
      target = tar.get();
    } else if (targetTimer.hasElapsed(0.2)) {
      System.out.println("no target");
      target = null;
    }
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yController.atSetpoint();
  }
}
