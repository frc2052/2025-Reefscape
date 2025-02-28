package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private boolean isAlignGoal;
  private boolean hasCoral;
  private boolean isIntaking;

  private static boolean regularNudge = true;
  private static Transform2d customNudgeAmt = new Transform2d(0, 0, new Rotation2d());

  public static RobotState getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotState();
    }

    return INSTANCE;
  }

  private RobotState() {}

  public void setIsIntaking(boolean isIntaking) {
    this.isIntaking = isIntaking;
  }

  public boolean getIsIntaking() {
    return isIntaking;
  }

  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public boolean getHasCoral() {
    return hasCoral;
  }

  public static boolean getRegularNudge() {
    return regularNudge;
  }

  // have to set back to true each time we apply a custom offset
  public static void setRegularNudge(boolean regular) {
    System.out.println("SET REGULAR NUDGE TO " + regular);
    regularNudge = regular;
  }

  public static void setCustomNudge(Transform2d nudgeamt) {
    setRegularNudge(false);
    customNudgeAmt = nudgeamt;
  }

  public static Transform2d getCustomNudge() {
    return customNudgeAmt;
  }

  public Pose2d getFieldToRobot() {
    if (drivetrainState.Pose != null) {
      return drivetrainState.Pose;
    }

    return MathHelpers.POSE_2D_ZERO;
  }

  public void addDrivetrainState(SwerveDriveState drivetrainState) {
    this.drivetrainState = drivetrainState;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return drivetrainState.Speeds;
  }

  public double distanceToAlignPose() {
    if (getAlignPose() == null) {
      return Double.POSITIVE_INFINITY;
    }
    return Math.abs(
        getAlignPose().getTranslation().getDistance(getFieldToRobot().getTranslation()));
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
        seenReefFace = AlignmentCommandFactory.idToReefFace(camTarget.fiducialId);

        goalAlignPose =
            getRegularNudge() == true
                ? AlignmentCommandFactory.reefIdToBranchWithNudge(
                    AlignmentCommandFactory.idToReefFace(camTarget.fiducialId), getAlignOffset())
                : AlignmentCommandFactory.reefIdToBranchCustomNudge(
                    AlignmentCommandFactory.idToReefFace(camTarget.fiducialId),
                    getAlignOffset(),
                    getCustomNudge());
      } else {
        if (poseAlignTimer.get() > 1.0) {
          goalAlignPose = null;
          seenReefFace = null;
        }
      }
    } else {
      if (poseAlignTimer.get() > 1.0) {
        goalAlignPose = null;
        seenReefFace = null;
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

  // public void reZeroAfterAuto() { // auto starts facing 180 of where it should
  //   Pose2d endAutoPose = drivetrainState.Pose; // store @ teleopInit to reset pose later
  //   Pose2d zeroingPose =
  //       new Pose2d(
  //           autoStartPose.getX(),
  //           autoStartPose.getY(),
  //           autoStartPose.getRotation().minus(new Rotation2d(Degrees.of(180))));
  //   DrivetrainSubsystem.getInstance().resetPose(zeroingPose);
  //   DrivetrainSubsystem.getInstance().seedFieldCentric();
  //   DrivetrainSubsystem.getInstance().resetPose(endAutoPose);
  // }

  public boolean getisAlignGoal() {
    return isAlignGoal;
  }

  public void setIsAlignGoal(boolean atGoal) {
    isAlignGoal = atGoal;
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
    // Logger.recordOutput("Swerve Module States", drivetrainState.ModuleStates);
    // Logger.recordOutput("Swerve Module Goals", drivetrainState.ModuleTargets);
    Logger.recordOutput("Current Pose", drivetrainState.Pose);
    Logger.recordOutput("Auto Start Pose", autoStartPose);
    Logger.recordOutput("Goal Align Pose", goalPose);
  }
}
