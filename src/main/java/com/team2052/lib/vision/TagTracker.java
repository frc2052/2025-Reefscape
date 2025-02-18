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
  private final RobotState robotState;
  private List<PhotonPipelineResult> latestResults;
  private double weight = 1.0;

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

  public void setWeight(double weight) {
    this.weight = weight;
  }

  public void pullData() {
    latestResults = photonCamera.getAllUnreadResults();
  }

  public List<PoseEstimate> getPoseEstimates() {
    List<PoseEstimate> estimates = new ArrayList<PoseEstimate>();
    for (int i = 0; i < latestResults.size(); i++) {
      PhotonPipelineResult result = latestResults.get(i);
      Optional<PoseEstimate> estimate = resultToPoseEstimate(result);
      if (estimate.isPresent()) {
        System.out.println("GETTING ESTIMATE");
        estimates.add(estimate.get());
      } else {
      }
    }

    return estimates;
  }

  public Optional<PhotonPipelineResult> getClosestTagToCamera() {
    PhotonPipelineResult closestTarget = null;
    for (PhotonPipelineResult r : latestResults) {
      if (r.hasTargets()) {
        PhotonTrackedTarget target = r.getBestTarget();
        if (closestTarget == null) {
          closestTarget = r;
        } else if (target.getBestCameraToTarget()
            == MathHelpers.getSmallestTransform(
                target.getBestCameraToTarget(),
                closestTarget.getBestTarget().getBestCameraToTarget())) {
          closestTarget = r;
        }
      }
    }

    return Optional.ofNullable(closestTarget);
  }

  private Optional<PoseEstimate> resultToPoseEstimate(PhotonPipelineResult result) {
    Optional<EstimatedRobotPose> photonData = poseEstimator.update(result);
    if (photonData.isPresent()) {
      System.out.println("DATA PRESENT");
      return Optional.of(
          new PoseEstimate(
              this.photonCamera.getName(), weight, photonData.get(), robotState.getFieldToRobot()));
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
