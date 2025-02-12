// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.commands.drive.alignment.AlignWithReefCommand.AlignLocation;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;

public class DistanceToVisionGoal extends Command {
  private VisionSubsystem vision = VisionSubsystem.getInstance();
  private Supplier<AlignLocation> alignLocSupplier;

  public DistanceToVisionGoal(Supplier<AlignLocation> alignLoc) {
    alignLocSupplier = alignLoc;
    // System.out.println("==== NEW DISTANCE TO VISION GOAL");
  }

  public double getDistanceToGoal(
      Supplier<AlignLocation> scoringLoc) { // accurate when we can see the tar
    Pose2d goalPose;
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tar.get().getBestTarget().fiducialId);
      if (tagPose.isPresent()) {
        goalPose = tagPose.get().toPose2d().transformBy(scoringLoc.get().transform);
      } else {
        // System.out.println("==== NO TAG POSE");
        return 50; // any number greater than our bound to switch to vision
      }
    } else {
      return 50;
    }

    Translation2d goalTranslation = new Translation2d(goalPose.getX(), goalPose.getY());
    return goalTranslation.getDistance(
        new Translation2d(
            RobotState.getInstance().getFieldToRobot().getX(),
            RobotState.getInstance().getFieldToRobot().getY()));
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double dist = getDistanceToGoal(alignLocSupplier);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
