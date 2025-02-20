// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team2052.lib.planners.AutoAlignPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotState;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TagTrackerType;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignWithFieldElementCommand extends DefaultDriveCommand {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final RobotState robotState = RobotState.getInstance();

  private Pose2d goalPose;
  private Supplier<AlignOffset> offsetSupplier;
  private AlignOffset selectedOffset;
  private AutoAlignPlanner planner;
  private DesiredElement desiredElement;
  private TargetFieldLocation fieldLocation;

  public AlignWithFieldElementCommand(
      TargetFieldLocation fieldLocation,
      Supplier<AlignOffset> offsetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);
    this.desiredElement = DesiredElement.SPECIFIC_REEF_FACE;
    this.offsetSupplier = offsetSupplier;
    this.fieldLocation = fieldLocation;

    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  public AlignWithFieldElementCommand(
      DesiredElement desiredElement,
      Supplier<AlignOffset> offsetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);
    this.desiredElement = desiredElement;
    this.fieldLocation = null;
    this.offsetSupplier = offsetSupplier;

    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    goalPose = null;
  }

  private void invalidCombination() {
    System.out.println(
        "The desired element "
            + desiredElement.toString()
            + " and the field location "
            + offsetSupplier.get().toString()
            + " do not match!");
    end(false);
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if (goalPose != null) {
      robotState.setGoalAlignment(goalPose);
      // return super.getSwerveRequest();
      return super.driveChassisSpeeds.withSpeeds(
          planner.calculate(robotState.getFieldToRobot(), goalPose));
    } else {
      return super.getSwerveRequest();
    }
  }

  @Override
  public void execute() {
    selectedOffset = offsetSupplier.get();
    if (desiredElement == DesiredElement.PROCESSOR) {
      if (selectedOffset != AlignOffset.PROCESSOR_MIDDLE_LOC) {
        invalidCombination();
      }
      vision.setPrimaryFocus(TagTrackerType.ALGAE_CAM);
    } else if (desiredElement == DesiredElement.REEF) {
      if (selectedOffset != AlignOffset.ALGAE_REEF_LOC
          && selectedOffset != AlignOffset.LEFT_REEF_LOC
          && selectedOffset != AlignOffset.MIDDLE_REEF_LOC
          && selectedOffset != AlignOffset.RIGHT_REEF_LOC) {
        invalidCombination();
      }
      vision.setPrimaryFocus(TagTrackerType.CORAL_REEF_CAM);
    } else if (desiredElement == DesiredElement.CORALSTATION) {
      if (selectedOffset != AlignOffset.LEFT_CORAL_STATION_LOC
          && selectedOffset != AlignOffset.RIGHT_CORAL_STATION_LOC) {
        invalidCombination();
      }
      vision.setPrimaryFocus(TagTrackerType.REAR_CAM);
    } else if (desiredElement == DesiredElement.SPECIFIC_REEF_FACE) {
      if (!fieldLocation.getIsReef()) {
        invalidCombination();
      }
      vision.setPrimaryFocus(TagTrackerType.CORAL_REEF_CAM);
    } else {
      vision.resetPrimaryFocus();
    }
    setGoalPose();
    super.execute();
  }

  protected void setGoalPose() {
    if (desiredElement == DesiredElement.PROCESSOR) {
      Optional<PhotonPipelineResult> tar = vision.getCameraClosestTarget(TagTrackerType.ALGAE_CAM);

      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        if (camTarget.fiducialId == 3 || camTarget.fiducialId == 16) {
          goalPose = TargetFieldLocation.PRC.getWithOffset(selectedOffset);
        } else {
          goalPose = null;
        }
      }
    } else if (desiredElement == DesiredElement.CORALSTATION) {
      Optional<PhotonPipelineResult> tar = vision.getCameraClosestTarget(TagTrackerType.REAR_CAM);

      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        if (camTarget.fiducialId == 1
            || camTarget.fiducialId == 13 && selectedOffset == AlignOffset.LEFT_CORAL_STATION_LOC) {
          goalPose = TargetFieldLocation.LCS.getWithOffset(selectedOffset);
        } else if (camTarget.fiducialId == 2
            || camTarget.fiducialId == 12
                && selectedOffset == AlignOffset.RIGHT_CORAL_STATION_LOC) {
          goalPose = TargetFieldLocation.RCS.getWithOffset(selectedOffset);
        } else {
          goalPose = null;
        }
      }
    } else if (desiredElement == DesiredElement.REEF) {
      Optional<PhotonPipelineResult> tar =
          vision.getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM);
      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();
        if (idToReefFace(camTarget.fiducialId) != null) {
          goalPose = reefIdToBranchWithNudge(idToReefFace(camTarget.fiducialId), selectedOffset);
        } else {
          goalPose = null;
        }
      }
    } else if (desiredElement == DesiredElement.SPECIFIC_REEF_FACE) {
      Optional<PhotonPipelineResult> tar =
          vision.getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM);

      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        if (idToReefFace(camTarget.fiducialId) == fieldLocation) {
          goalPose = reefIdToBranchWithNudge(fieldLocation, selectedOffset);
        } else {
          goalPose = null;
        }
      }
    }
  }

  private static Pose2d reefIdToBranchWithNudge(TargetFieldLocation location, AlignOffset offset) {
    if (offset == AlignOffset.LEFT_REEF_LOC) {
      return location.getWithTransform(offset.getTransform().plus(location.getLeftBranchNudge()));
    } else if (offset == AlignOffset.RIGHT_REEF_LOC) {
      return location.getWithTransform(offset.getTransform().plus(location.getRightBranchNudge()));
    }

    return location.getWithOffset(offset);
  }

  public static TargetFieldLocation idToReefFace(int id) {
    switch (id) {
      case 18:
        return TargetFieldLocation.AB;
      case 7:
        return TargetFieldLocation.AB;
      case 17:
        return TargetFieldLocation.CD;
      case 8:
        return TargetFieldLocation.CD;
      case 22:
        return TargetFieldLocation.EF;
      case 9:
        return TargetFieldLocation.EF;
      case 21:
        return TargetFieldLocation.GH;
      case 10:
        return TargetFieldLocation.GH;
      case 20:
        return TargetFieldLocation.IJ;
      case 11:
        return TargetFieldLocation.IJ;
      case 19:
        return TargetFieldLocation.KL;
      case 6:
        return TargetFieldLocation.KL;
      default:
        return null;
    }
  }

  @Override
  public boolean isFinished() {
    if (planner.getAutoAlignComplete()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interuppted) {
    planner.resetPlanner();
    vision.resetPrimaryFocus();
  }

  public static enum DesiredElement {
    PROCESSOR,
    REEF,
    CORALSTATION,
    SPECIFIC_REEF_FACE;
  }
}
