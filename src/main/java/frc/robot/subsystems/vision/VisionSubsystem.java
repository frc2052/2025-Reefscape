// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.vision.MultiTagPoseEstimate;
import com.team2052.lib.vision.TagTracker;
import com.team2052.lib.vision.VisionPoseAcceptor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.Camera0Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private RobotState robotState = RobotState.getInstance();
  List<MultiTagPoseEstimate> synchronizedVisionUpdates =
      Collections.synchronizedList(new ArrayList<MultiTagPoseEstimate>());

  private List<TagTracker> localizationTagTrackers = new ArrayList<TagTracker>();

  private TagTracker reefTagTracker =
      new TagTracker(Camera0Constants.TagTrackerConstants(), robotState);

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
        reefTagTracker);
  }

  public Optional<PhotonPipelineResult> getReefCamClosestTarget() {
    return reefTagTracker.getClosestTagToCamera();
  }

  private void updateLocalizationTrackers() {
    localizationTagTrackers.parallelStream().forEach(this::pullCameraData);
  }

  private void pullCameraData(TagTracker tagTracker) {
    synchronizedVisionUpdates.addAll(tagTracker.getAllResults());
  }

  private void updateEstimator(MultiTagPoseEstimate update) {
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

  @Override
  public void periodic() {
    synchronizedVisionUpdates.clear();

    updateLocalizationTrackers();

    synchronizedVisionUpdates.forEach(this::updateEstimator);
  }
}
