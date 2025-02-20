package com.team2052.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimate {
  private static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public final String cameraName;
  public final Pose3d estimatedPose;
  public final PoseStrategy strategyUsed;
  public List<PhotonTrackedTarget> targetsUsed;
  public final double timestampSeconds;
  public final double poseDifference;
  public final double cameraWeight;
  public double highestAmbiguity = 0.0;
  public double avgTagArea = 0.0;
  public double avgTagDistance = 0.0;

  public PoseEstimate(
      String cameraName,
      double cameraWeight,
      EstimatedRobotPose photonData,
      Pose2d currentRobotPose) {
    this.cameraName = cameraName;
    this.estimatedPose = photonData.estimatedPose;
    this.strategyUsed = photonData.strategy;
    this.targetsUsed = photonData.targetsUsed;
    this.timestampSeconds = photonData.timestampSeconds;
    this.cameraWeight = cameraWeight;
    if (targetsUsed != null) {
      for (PhotonTrackedTarget target : targetsUsed) {
        highestAmbiguity = Math.max(highestAmbiguity, target.getPoseAmbiguity());
        avgTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        avgTagArea += target.getArea();
      }
      avgTagDistance /= targetsUsed.size();
      avgTagArea /= targetsUsed.size();
    } else {
      targetsUsed = new ArrayList<PhotonTrackedTarget>();
    }

    // distance from current pose to vision estimated pose
    poseDifference =
        estimatedPose
            .getTranslation()
            .toTranslation2d()
            .getDistance(currentRobotPose.getTranslation());
  }

  public Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return (VecBuilder.fill(calculateXYStdDevs(), calculateXYStdDevs(), calculateHeadingStdDev()));
  }

  private double calculateXYStdDevs() {

    double xyStdDev =
        xyStdDevCoefficient * Math.pow(avgTagDistance, 2.0) / targetsUsed.size() * cameraWeight;
    // double xyStds = VisionConstants.XY_STDDEV;
    // if (targetsUsed.size() > 0) {
    //   // multiple targets detected
    //   if (targetsUsed.size() >= 2 && avgTagArea > 0.1) {
    //     xyStds = 0.2;
    //   }
    //   // 1 target with large area and close to estimated pose
    //   else if (avgTagArea > 0.8 && poseDifference < 0.5) {
    //     xyStds = 0.5;
    //   }
    //   // 1 target farther away and estimated pose is close
    //   else if (avgTagArea > 0.1 && poseDifference < 0.3) {
    //     xyStds = 1.0;
    //   } else if (targetsUsed.size() > 1) {
    //     xyStds = 1.2;
    //   } else {
    //     xyStds = 2.0;
    //   }
    // }

    return xyStdDev;
  }

  private double calculateHeadingStdDev() {

    // double headingStdDev =
    //     targetsUsed.size() > 2
    //         ? thetaStdDevCoefficient
    //             * Math.pow(avgTagDistance, 2.0)
    //             / targetsUsed.size()
    //             * cameraWeight
    //         : Double.POSITIVE_INFINITY;
    double headingStdDev = 1.0;
    if (highestAmbiguity < 0.05) {
      headingStdDev = 0.05;
    }
    return headingStdDev;
  }
}
