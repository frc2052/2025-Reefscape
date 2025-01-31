package com.team2052.lib.vision;

import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

  public Optional<PhotonPipelineResult> getClosestTagToCamera() {
    PhotonPipelineResult closestTarget = null;
    for (PhotonPipelineResult r : photonCamera.getAllUnreadResults()) {
      if (r.hasTargets()) {
        // System.out.println("======= HAS TARGETS" + r.hasTargets());
        PhotonTrackedTarget target = r.getBestTarget();
        if (closestTarget == null) {
          closestTarget = r;
        } else if (target.getBestCameraToTarget()
            == MathHelpers.getSmallestTransform(
                target.getBestCameraToTarget(),
                closestTarget.getBestTarget().getBestCameraToTarget())) {
          closestTarget = r;
        }
      } else {
        System.out.println("NO PHOTON PIPELINE RESULTS");
      }
    }

    return Optional.ofNullable(closestTarget);
  }

  public List<MultiTagPoseEstimate> getAllResults() {
    List<MultiTagPoseEstimate> estimates = new ArrayList<MultiTagPoseEstimate>();
    List<PhotonPipelineResult> latestResults = photonCamera.getAllUnreadResults();
    for (int i = 0; i < latestResults.size(); i++) {
      PhotonPipelineResult result = latestResults.get(i);
      Optional<MultiTagPoseEstimate> estimate = resultToMultiTag(result);
      if (estimate.isPresent()) {
        estimates.add(estimate.get());
      }
    }

    return estimates;
  }

  public Optional<MultiTagPoseEstimate> resultToMultiTag(PhotonPipelineResult result) {
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
    if (estimatedRobotPose.isPresent()) {
      return Optional.of(
          new MultiTagPoseEstimate(
              this.photonCamera.getName(), estimatedRobotPose.get(), robotState.getFieldToRobot()));
    }

    return Optional.empty();
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
