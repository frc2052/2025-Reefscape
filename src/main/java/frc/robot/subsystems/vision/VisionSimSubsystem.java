// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
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
  AprilTagFieldLayout fieldLayout = VisionConstants.APRIL_TAG_FIELD_LAYOUT;

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
    reefCameraProperties.setAvgLatencyMs(50);
    reefCameraProperties.setLatencyStdDevMs(5);

    reefCameraSim = new PhotonCameraSim(reefCam, reefCameraProperties);
    reefCameraSim.enableDrawWireframe(true);

    visionSim.addCamera(
        reefCameraSim, Constants.VisionConstants.Camera1Constants.ROBOT_TO_CAMERA_METERS);
  }

  @Override
  public void periodic() {
    Pose2d newOdometryPose = state.getFieldToRobot();
    updateVisionSimWithPose(newOdometryPose);
    Logger.recordOutput("Vision Sim Sees Target", isSeeingTarget());
    Logger.recordOutput("Current Sim ID", getCurrentTagID());
  }

  public void updateVisionSimWithPose(Pose2d pose) {
    visionSim.update(pose);
    Field2d debugField = visionSim.getDebugField();
    debugField.getObject("EstimatedRobot").setPose(pose);
  }

  public void getDebugField() { // Raw Stream @ localhost:1181, Processed Stream (w/ outlined tags) @ localhost:1182
    visionSim.getDebugField();
  }

  @SuppressWarnings("removal")
  public boolean isSeeingTarget() {
    PhotonPipelineResult result = reefCam.getLatestResult();
    if (result.hasTargets()) return true;
    return false;
  }

  @SuppressWarnings("removal")
  public Optional<PhotonPipelineResult> getReefCamClosestTarget() {
    PhotonPipelineResult result = reefCam.getLatestResult();
    return Optional.ofNullable(result);
  }

  public int getCurrentTagID() {
    Optional<PhotonPipelineResult> optionalResult = getReefCamClosestTarget();
    PhotonPipelineResult result = optionalResult.isPresent() ? optionalResult.get() : null;

    if (result == null || !result.hasTargets()) {
      return 0;
    }

    PhotonTrackedTarget target = result.getBestTarget();

    return target.getFiducialId();
  }
}
