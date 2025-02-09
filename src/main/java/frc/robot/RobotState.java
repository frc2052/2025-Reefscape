package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

@Logged(name = "RobotState")
public class RobotState {
  private SwerveDriveState drivetrainState = new SwerveDriveState();

  private boolean isReefTracking;
  private boolean isStationTracking;
  private boolean isProcessorTracking;

  private static RobotState INSTANCE;

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

  public ChassisSpeeds getChassisSpeeds(boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds = drivetrainState.Speeds;
    if (isFieldRelative) {
      return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getFieldToRobot().getRotation());
    } else {
      return chassisSpeeds;
    }
  }

  // tracking tags
  public void setReefTracking(boolean isReefTracking) {
    this.isReefTracking = isReefTracking;
  }

  public boolean getIsReefTracking() {
    return isReefTracking;
  }

  public void setStationTracking(boolean isStationTracking) {
    this.isStationTracking = isStationTracking;
  }

  public boolean getStationTracking() {
    return isStationTracking;
  }

  public void setProcessorTracking(boolean isProcessorTracking) {
    this.isProcessorTracking = isProcessorTracking;
  }

  public boolean setProcessorTracking() {
    return isProcessorTracking;
  }

  public void addDrivetrainState(SwerveDriveState drivetrainState) {
    this.drivetrainState = drivetrainState;
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
  }
}
