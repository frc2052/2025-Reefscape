// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSimSubsystem extends SubsystemBase {
  private static VisionSimSubsystem INSTANCE;

  private final PhotonCamera reefCam = new PhotonCamera("KrawlerCam_000");
  private final int resWidth = 1280;
  private final int resHeight = 800;
  AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  VisionSystemSim visionSim;
  PhotonCameraSim reefCameraSim;
  RobotState state = RobotState.getInstance();

  public static VisionSimSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSimSubsystem();
    }

    return INSTANCE;
  }

  public VisionSimSubsystem() {
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(fieldLayout);

    // Needs real properties
    var reefCameraProperties = new SimCameraProperties();
    reefCameraProperties.setCalibration(resWidth, resHeight, Rotation2d.fromDegrees(90));
    reefCameraProperties.setCalibError(0.35, 0.10);
    reefCameraProperties.setFPS(15);
    reefCameraProperties.setAvgLatencyMs(20);
    reefCameraProperties.setLatencyStdDevMs(5);

    reefCameraSim = new PhotonCameraSim(reefCam, reefCameraProperties);
    reefCameraSim.enableDrawWireframe(true);
    // Empty Transform3d represents robot to camera offset
    visionSim.addCamera(
        reefCameraSim, Constants.VisionConstants.Camera1Constants.ROBOT_TO_CAMERA_METERS);
  }

  @Override
  public void periodic() {
    Pose2d newOdometryPose = state.getFieldToRobot();
    updateVisionSimWithPose(newOdometryPose);
    Logger.recordOutput("Vision Sim Sees Target", isSeeingTarget());
  }

  public void updateVisionSimWithPose(Pose2d pose) {
    visionSim.update(pose);
    var debugField = visionSim.getDebugField();
    debugField.getObject("EstimatedRobot").setPose(pose);
  }

  public void getDebugField() {//go to localhost:1182 for the debug sim
    visionSim.getDebugField();
  }

  public boolean isSeeingTarget() {
    List<PhotonPipelineResult> results = reefCam.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        return true;
      }
    }
    return false;
  }

  public Optional<PhotonPipelineResult> getReefCamClosestTarget() {
    PhotonPipelineResult closestTarget = null;
    for (PhotonPipelineResult r : reefCam.getAllUnreadResults()) {
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
}
