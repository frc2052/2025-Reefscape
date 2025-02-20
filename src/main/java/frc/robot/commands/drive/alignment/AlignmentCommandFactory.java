package frc.robot.commands.drive.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.alignment.AlignWithFieldElementCommand.DesiredElement;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TagTrackerType;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;
import frc.robot.util.io.Dashboard;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignmentCommandFactory {
  private static final VisionSubsystem vision = VisionSubsystem.getInstance();
  private static final RobotState robotState = RobotState.getInstance();
  private static final ControlBoard controlBoard = ControlBoard.getInstance();

  public static Command getReefAlignmentCommand(AlignOffset offset) {
    if (offset != AlignOffset.LEFT_REEF_LOC
        && offset != AlignOffset.MIDDLE_REEF_LOC
        && offset != AlignOffset.RIGHT_REEF_LOC) {
      return invalidCombination(DesiredElement.REEF, offset);
    }

    vision.setPrimaryFocus(TagTrackerType.CORAL_REEF_CAM);

    Optional<PhotonPipelineResult> tar =
        vision.getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM);

    if (tar.isPresent()) {
      PhotonTrackedTarget camTarget = tar.get().getBestTarget();
      if (idToReefFace(camTarget.fiducialId) != null) {
        return new DriveToPose(
            () -> reefIdToBranchWithNudge(idToReefFace(camTarget.fiducialId), offset),
            robotState::getFieldToRobot,
            controlBoard::getThrottle,
            controlBoard::getStrafe,
            controlBoard::getRotation);
      } else {
        return getDefaultDriveCommand();
      }
    } else {
      return getDefaultDriveCommand();
    }
  }

  public static Command getSpecificReefAlignmentCommand(
      AlignOffset offset, TargetFieldLocation fieldLocation) {
    if (offset != AlignOffset.LEFT_REEF_LOC
        && offset != AlignOffset.MIDDLE_REEF_LOC
        && offset != AlignOffset.RIGHT_REEF_LOC) {
      return invalidCombination(DesiredElement.REEF, offset);
    }

    vision.setPrimaryFocus(TagTrackerType.CORAL_REEF_CAM);

    Optional<PhotonPipelineResult> tar =
        vision.getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM);

    if (tar.isPresent()) {
      PhotonTrackedTarget camTarget = tar.get().getBestTarget();

      if (idToReefFace(camTarget.fiducialId) == fieldLocation) {
        return new DriveToPose(
            () -> reefIdToBranchWithNudge(idToReefFace(camTarget.fiducialId), offset),
            robotState::getFieldToRobot,
            controlBoard::getThrottle,
            controlBoard::getStrafe,
            controlBoard::getRotation);
      } else {
        return getDefaultDriveCommand();
      }
    } else {
      return getDefaultDriveCommand();
    }
  }

  private static Command getDefaultDriveCommand() {
    return new DefaultDriveCommand(
        controlBoard::getThrottle,
        controlBoard::getStrafe,
        controlBoard::getRotation,
        Dashboard.getInstance()::isFieldCentric);
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

  private static Command invalidCombination(DesiredElement desiredElement, AlignOffset offset) {
    return new PrintCommand(
        "The desired element "
            + desiredElement.toString()
            + " and the field location "
            + offset.toString()
            + " do not match!");
  }
}
