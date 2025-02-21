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
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
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
  // private TagTracker rearTagTracker =
  //     new TagTracker(RearCameraConstants.TagTrackerConstants(), robotState);

  private static VisionSubsystem INSTANCE;

  public static VisionSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSubsystem();
    }

    return INSTANCE;
  }

  private VisionSubsystem() {
    Collections.addAll(localizationTagTrackers, reefTagTracker, algaeTagTracker);
  }

  public Optional<PhotonPipelineResult> getCameraClosestTarget(TagTrackerType trackerType) {
    return getCameraClosestTarget(
        trackerType, Distance.ofBaseUnits(Double.POSITIVE_INFINITY, Meters));
  }

  public boolean getCoralCameraHasTarget() {
    return getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM, Meters.of(1.5)).isPresent();
  }

  // public boolean getStationCameraHasTarget() {
  //   return getCameraClosestTarget(TagTrackerType.REAR_CAM, Meters.of(1.5)).isPresent();
  // }

  public Optional<PhotonPipelineResult> getCameraClosestTarget(
      TagTrackerType trackerType, Distance maxDistance) {
    switch (trackerType) {
      case CORAL_REEF_CAM:
        if (maxDistance(reefTagTracker.getClosestTagToCamera(), maxDistance).isPresent()) {
          System.out.println("====== SEES REEF TAG");
        }
        return maxDistance(reefTagTracker.getClosestTagToCamera(), maxDistance);
      case ALGAE_CAM:
        return maxDistance(algaeTagTracker.getClosestTagToCamera(), maxDistance);
        // case REAR_CAM:
        //   return maxDistance(rearTagTracker.getClosestTagToCamera(), maxDistance);
      default:
        return Optional.empty();
    }
  }

  private Optional<PhotonPipelineResult> maxDistance(
      Optional<PhotonPipelineResult> photonResult, Distance maxDistance) {
    if (photonResult.isPresent()) {
      PhotonPipelineResult result = photonResult.get();
      if (result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm()
          < maxDistance.in(Meters)) {
        return reefTagTracker.getClosestTagToCamera();
      }
    }
    return Optional.empty();
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
    }
  }

  public AprilTagFieldLayout getCameraLayout(TagTrackerType trackerType) {
    switch (trackerType) {
      case CORAL_REEF_CAM:
        return reefTagTracker.constants.tagLayout;
      case ALGAE_CAM:
        return algaeTagTracker.constants.tagLayout;
        // case REAR_CAM:
        //   return rearTagTracker.constants.tagLayout;
      default:
        return FieldConstants.DEFAULT_APRIL_TAG_LAYOUT_TYPE.layout;
    }
  }

  public void setPrimaryFocus(TagTrackerType trackerType) {
    switch (trackerType) {
      case CORAL_REEF_CAM:
        reefTagTracker.setWeight(0.6);
        algaeTagTracker.setWeight(1.2);
        // rearTagTracker.setWeight(1.2);
      case ALGAE_CAM:
        reefTagTracker.setWeight(1.2);
        algaeTagTracker.setWeight(0.6);
        // rearTagTracker.setWeight(1.2);
        // case REAR_CAM:
        //   reefTagTracker.setWeight(1.2);
        //   algaeTagTracker.setWeight(1.2);
        //   rearTagTracker.setWeight(0.6);
    }
  }

  public void resetPrimaryFocus() {
    reefTagTracker.setWeight(1.0);
    algaeTagTracker.setWeight(1.0);
    // rearTagTracker.setWeight(1.0);
  }

  @Override
  public void periodic() {
    synchronizedVisionUpdates.clear();

    updateTagTrackers();

    synchronizedVisionUpdates.forEach(this::updateEstimator);
    robotState.seenReefFace(getCameraClosestTarget(robotState.getPrimaryCameraFocus()));
  }

  public enum TagTrackerType {
    CORAL_REEF_CAM,
    ALGAE_CAM;
    // REAR_CAM;
  }
}
