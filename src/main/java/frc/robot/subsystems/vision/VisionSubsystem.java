// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.vision.TagTracker;
import com.team2052.lib.vision.VisionPoseAcceptor;
import com.team2052.lib.vision.VisionUpdate;

import edu.wpi.first.math.geometry.Translation2d;
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
  List<VisionUpdate> synchronizedVisionUpdates =
      Collections.synchronizedList(new ArrayList<VisionUpdate>());

  private List<TagTracker> tagTrackers = new ArrayList<TagTracker>();

  private TagTracker reefTagTracker = new TagTracker(Camera0Constants.TagTrackerConstants(), robotState);

  private static VisionSubsystem INSTANCE;

  public static VisionSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSubsystem();
    }

    return INSTANCE;
  }
  /** Creates a new VisionSubsystem. */
  private VisionSubsystem() {
  }

  public Translation2d getReefCamResult() {
    return reefTagTracker.getClosestTagTransform();
  }

  private void update() {
    tagTrackers.parallelStream().forEach(this::pullCameraData);
  }

  private void pullCameraData(TagTracker tagTracker) {
    List<PhotonPipelineResult> latestResults = tagTracker.photonCamera.getAllUnreadResults();
    for (int i = 0; i < latestResults.size(); i++) {
      Optional<VisionUpdate> visionUpdate = tagTracker.resultToVisionUpdate(latestResults.get(i));

      if (visionUpdate.isPresent()) {
        synchronizedVisionUpdates.add(visionUpdate.get());
        Logger.recordOutput(tagTracker.getName() + " has update", true);
      } else {
        Logger.recordOutput(tagTracker.getName() + " has update", false);
      }
    }
  }

  private void updateEstimator(VisionUpdate update) {
    if (VisionPoseAcceptor.shouldAccept(
        update,
        MathHelpers.norm(drivetrain.getCurrentRobotChassisSpeeds()),
        robotState.getFieldToRobot(),
        DriverStation.isAutonomous())) {
      DrivetrainSubsystem.getInstance().addVisionUpdate(update);
    }
  }

  @Override
  public void periodic() {
    synchronizedVisionUpdates.clear();

    update();

    synchronizedVisionUpdates.forEach(this::updateEstimator);
  }
}
