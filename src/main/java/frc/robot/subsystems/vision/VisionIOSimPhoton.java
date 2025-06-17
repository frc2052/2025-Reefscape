// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

public class VisionIOSimPhoton implements VisionIO {
    private static VisionIOSimPhoton INSTANCE;

    private final PhotonCamera reefCam = new PhotonCamera("KrawlerCam_000");
    private final int resWidth = 1280;
    private final int resHeight = 800;
    private final AprilTagFieldLayout fieldLayout = FieldConstants.DEFAULT_APRIL_TAG_LAYOUT;

    private final VisionSystemSim visionSim;
    private final PhotonCameraSim reefCameraSim;
    private final RobotState state = RobotState.getInstance();
    private Pose2d lastPose;

    public static VisionIOSimPhoton getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionIOSimPhoton();
        }

        return INSTANCE;
    }

    public VisionIOSimPhoton() {
        lastPose = new Pose2d();
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

        visionSim.addCamera(reefCameraSim, CoralReefCameraConstants.ROBOT_TO_CAMERA);
    }

    @Override
    public void update() {
        Pose2d newOdometryPose = state.getFieldToRobot();
        updateVisionSimWithPose(newOdometryPose);
        Logger.recordOutput("Vision Sim Sees Target", isSeeingTarget());
        Logger.recordOutput("Current Sim ID", getAllVisibleTagIDs());
        if (getCurrentTagID() != 0) {
            Logger.recordOutput(
                    "Current Sim offset from tag", new Transform2d(lastPose, newOdometryPose).getTranslation());
        }
        if (getCurrentTag().bestCameraToTarget != null) {
            Logger.recordOutput(
                    "Current Sim TRANSFORM ROBOT TO TARGET",
                    getCurrentTag()
                            .bestCameraToTarget
                            .plus(CoralReefCameraConstants.ROBOT_TO_CAMERA.inverse())
                            .getTranslation()
                            .toTranslation2d());
        }
    }

    public void updateVisionSimWithPose(Pose2d pose) {
        visionSim.update(pose);
        Field2d debugField = visionSim.getDebugField();
        if (getCurrentTagID() != 0) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(getCurrentTagID());
            if (tagPose.isPresent()) {
                lastPose = tagPose.get().toPose2d();
            } else {
            }
        }
        debugField.getObject("EstimatedRobot").setPose(pose);
    }

    public void getDebugField() { // Raw Stream @ localhost:1181, Processed Stream (w/ outlined tags) @
        // localhost:1182
        visionSim.getDebugField();
    }

    public boolean isSeeingTarget() {
        PhotonPipelineResult result = reefCam.getAllUnreadResults().get(0); // messy grab, but don't really care
        if (result.hasTargets()) return true;
        return false;
    }

    public Optional<PhotonPipelineResult> getReefCamClosestTarget() {
        PhotonPipelineResult result = reefCam.getAllUnreadResults().get(0); // messy grab, but don't really care
        return Optional.ofNullable(result);
    }

    public boolean hasReefTarget() {
        return false; // getCurrentTag()
    }

    public PhotonTrackedTarget getCurrentTag() {
        Optional<PhotonPipelineResult> optionalResult = getReefCamClosestTarget();
        PhotonPipelineResult result = optionalResult.isPresent() ? optionalResult.get() : null;

        if (result == null || !result.hasTargets()) {
            return new PhotonTrackedTarget();
        }

        PhotonTrackedTarget target = result.getBestTarget();

        return target;
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
