// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.CoralReefCameraConstants;
import frc.robot.RobotState;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
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
  private final AprilTagFieldLayout fieldLayout =
      FieldConstants.DEFAULT_APRIL_TAG_LAYOUT_TYPE.layout;

  private final VisionSystemSim visionSim;
  private final PhotonCameraSim reefCameraSim;
  private final RobotState state = RobotState.getInstance();

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

    visionSim.addCamera(reefCameraSim, CoralReefCameraConstants.ROBOT_TO_CAMERA_METERS);
  }

  @Override
  public void periodic() {
    Pose2d newOdometryPose = state.getFieldToRobot();
    updateVisionSimWithPose(newOdometryPose);
    Logger.recordOutput("Vision Sim Sees Target", isSeeingTarget());
    Logger.recordOutput("Current Sim ID", getAllVisibleTagIDs());
  }

  public void updateVisionSimWithPose(Pose2d pose) {
    visionSim.update(pose);
    Field2d debugField = visionSim.getDebugField();
    debugField.getObject("EstimatedRobot").setPose(pose);
  }

  public void
      getDebugField() { // Raw Stream @ localhost:1181, Processed Stream (w/ outlined tags) @
    // localhost:1182
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

  public int[] getAllVisibleTagIDs() {
    List<PhotonPipelineResult> results = reefCam.getAllUnreadResults();
    ArrayList<Integer> tagIDs = new ArrayList<>();
    List<PhotonTrackedTarget> targets;
    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        targets = result.getTargets();
        for (PhotonTrackedTarget target : targets) {
          tagIDs.add(target.getFiducialId());
        }
      }
    }

    return tagIDs.stream().mapToInt(Integer::intValue).toArray();
  }
}
