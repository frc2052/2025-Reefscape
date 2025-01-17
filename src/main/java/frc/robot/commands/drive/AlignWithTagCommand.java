// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team2052.lib.planners.AutoAlignPlanner;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignWithTagCommand extends DefaultDriveCommand {
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final RobotState robotState = RobotState.getInstance();
  private PhotonTrackedTarget target;
  private AlignLocation scoringLocation;
  private AutoAlignPlanner planner;
  private Timer targetTimer = new Timer();

  private SwerveRequest.ApplyFieldSpeeds drive =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithTagCommand(
      AlignLocation scoringLocation,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) { // add enum supplier for scoring position, left middle or right
    super(xSupplier, ySupplier, rotationSupplier, () -> false);

    this.scoringLocation = scoringLocation;
    planner = new AutoAlignPlanner();
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if (target != null) {
      // System.out.println(
      //     "Camera Transform Rotation"
      //         + target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
      // return super.getSwerveRequest();
      return drive.withSpeeds(
          planner.calculate(
              target.getBestCameraToTarget(),
              scoringLocation.goalTransform,
              robotState.getFieldToRobot()));
    } else {
      return super.getSwerveRequest();
    }
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

  @Override
  public boolean isFinished() {
    return planner.getAutoAlignComplete();
  }

  // public SnapLocation getDirection() {
  //   SnapLocation location;
  //   switch (target.fiducialId) {
  //     case 18:
  //       location = SnapLocation.ReefAB;
  //       break;
  //     case 17:
  //       location = SnapLocation.ReefCD;
  //       break;
  //     case 22:
  //       location = SnapLocation.ReefEF;
  //       break;
  //     case 21:
  //       location = SnapLocation.ReefGH;
  //       break;
  //     case 20:
  //       location = SnapLocation.ReefIJ;
  //       break;
  //     case 19:
  //       location = SnapLocation.ReefKL;
  //       break;
  //     case 10:
  //       location = SnapLocation.ReefGH;
  //       break;
  //     case 11:
  //       location = SnapLocation.ReefIJ;
  //       break;
  //     case 6:
  //       location = SnapLocation.ReefKL;
  //       break;
  //     case 7:
  //       location = SnapLocation.ReefAB;
  //       break;
  //     case 8:
  //       location = SnapLocation.ReefCD;
  //       break;
  //     case 9:
  //       location = SnapLocation.ReefEF;
  //       break;
  //     default:
  //       location = SnapLocation.FORWARD;
  //   }

  //   return location;
  // }

  public enum AlignLocation {
    LEFT(new Transform2d(0.2, 0.5, new Rotation2d())),
    MIDDLE(new Transform2d(0.2, 0.01, Rotation2d.fromDegrees(177))),
    RIGHT(new Transform2d(0.2, -0.5, new Rotation2d()));

    private Transform2d goalTransform;

    private AlignLocation(Transform2d gt) {
      this.goalTransform = gt;
    }

    public Transform2d getTransform2d() {
      return goalTransform;
    }
  }
}
