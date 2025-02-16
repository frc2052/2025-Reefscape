// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.vision.PoseEstimate;
import com.team2052.lib.vision.TagTracker;
import com.team2052.lib.vision.VisionPoseAcceptor;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.AlgaeReefCameraConstants;
import frc.robot.Constants.VisionConstants.CoralReefCameraConstants;
import frc.robot.Constants.VisionConstants.RearCameraConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private RobotState robotState = RobotState.getInstance();
  List<PoseEstimate> synchronizedVisionUpdates =
      Collections.synchronizedList(new ArrayList<PoseEstimate>());

  private List<TagTracker> localizationTagTrackers = new ArrayList<TagTracker>();

  private TagTracker reefTagTracker =
      new TagTracker(CoralReefCameraConstants.TagTrackerConstants(), robotState);
  private TagTracker algaeTagTracker =
      new TagTracker(AlgaeReefCameraConstants.TagTrackerConstants(), robotState);
  private TagTracker rearTagTracker =
      new TagTracker(RearCameraConstants.TagTrackerConstants(), robotState);

  private static VisionSubsystem INSTANCE;

  public static VisionSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSubsystem();
    }

    return INSTANCE;
  }

  private VisionSubsystem() {
    Collections.addAll(
        localizationTagTrackers,
        // new TagTracker(Camera1Constants.TagTrackerConstants(), robotState),
        reefTagTracker,
        algaeTagTracker);
  }

  public Optional<PhotonPipelineResult> getReefCamClosestTarget() {
    return reefTagTracker.getClosestTagToCamera();
  }

  public Optional<PhotonPipelineResult> getReefCamClosestTarget(Distance minDistance) {
    if (reefTagTracker.getClosestTagToCamera().isPresent()) {
      PhotonPipelineResult result = reefTagTracker.getClosestTagToCamera().get();
      if (result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm()
          < minDistance.in(Meters)) {
        return reefTagTracker.getClosestTagToCamera();
      }
    }
    return Optional.empty();
  public Optional<PhotonPipelineResult> getAlgaeCamTarget() {
    return algaeTagTracker.getClosestTagToCamera();
  }

  public Optional<PhotonPipelineResult> getCoralStationTarget() {
    return coralStationTracker.getClosestTagToCamera();
  }

  private void updateTagTrackers() {
    localizationTagTrackers.parallelStream().forEach(this::pullCameraData);
  }

  private void pullCameraData(TagTracker tagTracker) {
    tagTracker.pullData();
    synchronizedVisionUpdates.addAll(tagTracker.getPoseEstimates());
  }

  private void updateEstimator(PoseEstimate update) {
    if (VisionPoseAcceptor.shouldAccept(
        update,
        MathHelpers.chassisSpeedsNorm(drivetrain.getCurrentRobotChassisSpeeds()),
        robotState.getFieldToRobot(),
        DriverStation.isAutonomous())) {
      DrivetrainSubsystem.getInstance().addVisionUpdate(update);

      Logger.recordOutput(update.cameraName + " update valid", true);
    } else {

      Logger.recordOutput(update.cameraName + " update valid", false);
    }
  }

  public AprilTagFieldLayout getCameraLayout(TagTrackerType trackerType) {
    switch (trackerType) {
      case CORAL_REEF:
        return reefTagTracker.constants.tagLayout;
      case ALGAE_REEF:
        return algaeTagTracker.constants.tagLayout;
      case REAR:
        return rearTagTracker.constants.tagLayout;
      default:
        return FieldConstants.DEFAULT_APRIL_TAG_LAYOUT_TYPE.layout;
    }
  }

  @Override
  public void periodic() {
    synchronizedVisionUpdates.clear();

    updateTagTrackers();

    synchronizedVisionUpdates.forEach(this::updateEstimator);
  }

  public enum TagTrackerType {
    CORAL_REEF,
    ALGAE_REEF,
    REAR;
  }
}
