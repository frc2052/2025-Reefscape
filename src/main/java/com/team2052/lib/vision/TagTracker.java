package com.team2052.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.team2052.lib.helpers.MathHelpers;

public class TagTracker {
  private PhotonPoseEstimator poseEstimator;
  public PhotonCamera photonCamera;
  public TagTrackerConstants constants;
  public RobotState robotState;

  public TagTracker(TagTrackerConstants camConstants, RobotState robotState) {
    this.constants = camConstants;
    this.photonCamera = new PhotonCamera(camConstants.name);
    this.poseEstimator =
        new PhotonPoseEstimator(
            camConstants.tagLayout, camConstants.strategy, camConstants.robotToCamera);
    this.robotState = robotState;
  }

  public String getName() {
    return photonCamera.getName();
  }

  public Translation2d getClosestTagTransform() {
    Translation2d bestTranslation = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
    for(PhotonPipelineResult r : photonCamera.getAllUnreadResults()) {
      if(r.getBestTarget().bestCameraToTarget.getTranslation().toTranslation2d().getNorm() < bestTranslation.getNorm()) {
        bestTranslation = r.getBestTarget().bestCameraToTarget.getTranslation().toTranslation2d();
      }
    }

    return bestTranslation;
  }

  public Optional<VisionUpdate> resultToVisionUpdate(PhotonPipelineResult result) {
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
    if (estimatedRobotPose.isPresent()) {
      return Optional.of(
          new VisionUpdate(
              this.photonCamera.getName(), estimatedRobotPose.get(), robotState.getFieldToRobot()));
    } else {
      return Optional.empty();
    }
  }

  public static class TagTrackerConstants {
    public final String name;
    public final Transform3d robotToCamera;
    public final AprilTagFieldLayout tagLayout;
    public final PoseStrategy strategy;

    public TagTrackerConstants(
        String name,
        Transform3d robotToCamera,
        AprilTagFieldLayout tayLayout,
        PoseStrategy strategy) {
      this.name = name;
      this.robotToCamera = robotToCamera;
      this.tagLayout = tayLayout;
      this.strategy = strategy;
    }
  }
}
