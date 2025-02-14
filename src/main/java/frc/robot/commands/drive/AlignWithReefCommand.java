// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team2052.lib.planners.AutoAlignPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.controlboard.PositionSuperstructure.ReefSubSide;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AimingCalculator;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignWithReefCommand extends DefaultDriveCommand {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final RobotState robotState = RobotState.getInstance();

  private PhotonTrackedTarget target;

  private Pose2d goalPose;
  private Supplier<ReefSubSide> scoringLocation;

  private AutoAlignPlanner planner;

  private SwerveRequest.ApplyFieldSpeeds drive =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithReefCommand(
      Supplier<ReefSubSide> scoringLocation,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);

    this.scoringLocation = scoringLocation;
    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if (goalPose != null) {
      return drive.withSpeeds(planner.calculate(robotState.getFieldToRobot(), goalPose));
    } else {
      return super.getSwerveRequest();
    }
  }

  @Override
  public void initialize() {
    goalPose = null;
    robotState.setReefTracking(true);
  }

  @Override
  public void execute() {
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      Logger.recordOutput("Target for Alignment", true);
      target = tar.get().getBestTarget();
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);
      if (tagPose.isPresent()) {
        goalPose =
            AimingCalculator.horizontalAjustment(
                Meters.of(scoringLocation.get().transform.getY()),
                AimingCalculator.scaleFromReef(
                    tagPose.get().toPose2d(),
                    Meters.of(scoringLocation.get().transform.getX()),
                    robotState.isRedAlliance()));
        Logger.recordOutput("Tag Pose Present", true);
      } else {
        Logger.recordOutput("Tag Pose Present", false);
        goalPose = null;
      }
    } else {
      Logger.recordOutput("Target for Alignment", false);
      target = null;
    }
    super.execute();
  }

  public void setGoal(Pose2d goalPose) {
    this.goalPose = goalPose;
  }

  public ReefSubSide getScoringLocation() {
    return scoringLocation.get();
  }

  @Override
  public boolean isFinished() {
    if (planner.getAutoAlignComplete()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interuppted) {
    goalPose = null;
    target = null;
    planner.resetPlanner();
    robotState.setReefTracking(false);
  }
}
