// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import com.ctre.phoenix6.swerve.SwerveModule;
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
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignWithFieldElementCommand extends DefaultDriveCommand {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final RobotState robotState = RobotState.getInstance();

  private Pose2d goalPose;
  private AlignOffset offset;
  private AutoAlignPlanner planner;
  private DesiredElement desiredElement;
  private TargetFieldLocation fieldLocation;

  private SwerveRequest.ApplyFieldSpeeds drive =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithFieldElementCommand(
      TargetFieldLocation fieldLocation,
      AlignOffset offset,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);
    this.desiredElement = DesiredElement.SPECIFIC_REEF_FACE;
    this.offset = offset;
    this.fieldLocation = fieldLocation;

    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  public AlignWithFieldElementCommand(
      DesiredElement desiredElement,
      AlignOffset offset,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);
    this.desiredElement = desiredElement;
    this.fieldLocation = null;
    this.offset = offset;

    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    goalPose = null;

    if (desiredElement == DesiredElement.PROCESSOR) {
      vision.setPrimaryFocus(TagTrackerType.ALGAE_CAM);
    } else if (desiredElement == DesiredElement.REEF
        || desiredElement == DesiredElement.SPECIFIC_REEF_FACE) {
      vision.setPrimaryFocus(TagTrackerType.CORAL_REEF_CAM);
    } else if (desiredElement == DesiredElement.CORALSTATION) {
      vision.setPrimaryFocus(TagTrackerType.REAR_CAM);
    } else {
      vision.resetPrimaryFocus();
    }
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if (goalPose != null) {
      return drive.withSpeeds(planner.calculate(robotState.getFieldToRobot(), goalPose));
    } else {
      return super.getSwerveRequest();
    }
  }

  @Override
  public void execute() {
    setGoalPose();
    super.execute();
  }

  protected void setGoalPose() {
    Logger.recordOutput("Target for Alignment: ", desiredElement.toString());
    if (desiredElement == DesiredElement.PROCESSOR) {
      Optional<PhotonPipelineResult> tar = vision.getCameraClosestTarget(TagTrackerType.ALGAE_CAM);

      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        System.out.println(this.getName() + " COMMAND SEES TAG: " + camTarget.fiducialId);

        if (camTarget.fiducialId == 3 || camTarget.fiducialId == 16) {
          goalPose = TargetFieldLocation.PRC.getWithOffset(offset);
        } else {
          goalPose = null;
        }
      }
    } else if (desiredElement == DesiredElement.CORALSTATION) {
      Optional<PhotonPipelineResult> tar = vision.getCameraClosestTarget(TagTrackerType.REAR_CAM);

      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        System.out.println(this.getName() + " COMMAND SEES TAG: " + camTarget.fiducialId);

        if (camTarget.fiducialId == 1
            || camTarget.fiducialId == 13 && offset == AlignOffset.LEFT_CORAL_STATION_LOC) {
          goalPose = TargetFieldLocation.LCS.getWithOffset(offset);
        } else if (camTarget.fiducialId == 2
            || camTarget.fiducialId == 12 && offset == AlignOffset.RIGHT_CORAL_STATION_LOC) {
          goalPose = TargetFieldLocation.RCS.getWithOffset(offset);
        } else {
          goalPose = null;
        }
      }
    } else if (desiredElement == DesiredElement.REEF) {
      Optional<PhotonPipelineResult> tar =
          vision.getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM);
      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        System.out.println(this.getName() + " COMMAND SEES TAG: " + camTarget.fiducialId);

        switch (camTarget.fiducialId) {
          case 18:
          case 7:
            goalPose = TargetFieldLocation.AB.getWithOffset(offset);
            break;
          case 17:
          case 8:
            goalPose = TargetFieldLocation.CD.getWithOffset(offset);
            break;
          case 22:
          case 9:
            goalPose = TargetFieldLocation.EF.getWithOffset(offset);
            break;
          case 21:
          case 10:
            goalPose = TargetFieldLocation.GH.getWithOffset(offset);
            break;
          case 20:
          case 11:
            goalPose = TargetFieldLocation.IJ.getWithOffset(offset);
            break;
          case 19:
          case 6:
            goalPose = TargetFieldLocation.KL.getWithOffset(offset);
            break;
          default:
            goalPose = null;
            break;
        }
      }
    } else if (desiredElement == DesiredElement.SPECIFIC_REEF_FACE) {
      Optional<PhotonPipelineResult> tar =
          vision.getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM);

      if (tar.isPresent()) {
        PhotonTrackedTarget camTarget = tar.get().getBestTarget();

        if (camTarget.fiducialId == fieldLocation.getTagID()) {
          goalPose = fieldLocation.getWithOffset(offset);
        } else {
          goalPose = null;
        }
      }
    }
  }

  protected int getSpecificReefSideTag() {
    return 0;
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
