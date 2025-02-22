package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.subsystems.vision.VisionSubsystem.TagTrackerType;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotState {
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private AlignOffset selectedAlignOffset = AlignOffset.MIDDLE_REEF_LOC;
  private Pose2d goalPose;
  private Pose2d autoStartPose;
  private static RobotState INSTANCE;
  private TargetFieldLocation seenReefFace;
  private TargetFieldLocation desiredReefFace;
  private Pose2d goalAlignPose;
  private Timer poseAlignTimer = new Timer();

  public static RobotState getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotState();
    }

    return INSTANCE;
  }

  private RobotState() {}

  public Pose2d getFieldToRobot() {
    if (drivetrainState.Pose != null) {
      return drivetrainState.Pose;
    }

    return MathHelpers.POSE_2D_ZERO;
  }

  public void addDrivetrainState(SwerveDriveState drivetrainState) {
    this.drivetrainState = drivetrainState;
  }

  public void setAlignOffset(AlignOffset offset) {
    System.out.println("NEW OFFSET " + offset.transform.getY());
    selectedAlignOffset = offset;
  }

  public AlignOffset getAlignOffset() {
    return selectedAlignOffset;
  }

  public void seenReefFace(Optional<PhotonPipelineResult> result) {
    if (result.isPresent()) {
      poseAlignTimer.restart();
      PhotonTrackedTarget camTarget = result.get().getBestTarget();
      if (AlignmentCommandFactory.idToReefFace(camTarget.fiducialId) != null
          && AlignmentCommandFactory.idToReefFace(camTarget.fiducialId).getIsReef()) {
        goalAlignPose =
            AlignmentCommandFactory.reefIdToBranchWithNudge(
                AlignmentCommandFactory.idToReefFace(camTarget.fiducialId), getAlignOffset());
      } else {
        if (poseAlignTimer.get() > 1.0) {
          goalAlignPose = null;
        }
      }
    } else {
      if (poseAlignTimer.get() > 1.0) {
        goalAlignPose = null;
      }
    }
  }

  public boolean getHasAlignPose() {
    System.out.println("NO ALIGN POSE*********************");
    return goalAlignPose != null;
  }

  public Pose2d getAlignPose() {
    return goalAlignPose;
  }

  public void setDesiredReefFace(TargetFieldLocation desiredReefFace) {
    this.desiredReefFace = desiredReefFace;
  }

  public boolean desiredReefFaceIsSeen() {
    if (seenReefFace == null || desiredReefFace == null) {
      return false;
    }

    return desiredReefFace.getTagID() == seenReefFace.getTagID();
  }

  public TagTrackerType getPrimaryCameraFocus() {
    return TagTrackerType.CORAL_REEF_CAM;
  }

  public void setGoalAlignment(Pose2d goalPose) {
    this.goalPose = goalPose;
  }

  public void setAutoStartPose(Pose2d startPose) {
    this.autoStartPose = startPose;
  }

  /**
   * Returns true if the robot is on red alliance.
   *
   * @return True if the robot is on red alliance.
   */
  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
    } else {
      return false;
    }
  }

  public void output() {
    Logger.recordOutput("Swerve Module States", drivetrainState.ModuleStates);
    Logger.recordOutput("Swerve Module Goals", drivetrainState.ModuleTargets);
    Logger.recordOutput("Current Pose", drivetrainState.Pose);
    Logger.recordOutput("Auto Start Pose", autoStartPose);
    Logger.recordOutput("Goal Align Pose", goalPose);
  }
}
