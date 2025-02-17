package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private SwerveDriveState drivetrainState = new SwerveDriveState();

  private boolean isReefTracking;
  private boolean hasCoral;

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

  public void setFieldLocation(fieldLocation location){
    fieldLocation = location;
  }

  public ChassisSpeeds getChassisSpeeds(boolean isFieldRelative) {
    ChassisSpeeds chassisSpeeds = drivetrainState.Speeds;
    if (isFieldRelative) {
      return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getFieldToRobot().getRotation());
    } else {
      return chassisSpeeds;
    }
  }

  public void setReefTracking(boolean isReefTracking) {
    this.isReefTracking = isReefTracking;
  }

  public boolean getIsReefTracking() {
    return isReefTracking;
  }

  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public boolean getHasCoral() {
    return hasCoral;
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

  public enum fieldLocation {
    HP(Translation2d(),Translation2d()),
    Reef(Translation2d(),Translation2d()),
    Barge(Translation2d(),Translation2d()),
    Processor(Translation2d(),Translation2d()),
    Travel (Translation2d(null,null),Translation2d(null,null));

    public final Pose2d bluePose; 
    public final Pose2d redPose; 

    private fieldLocation(Translation2d bluePose,Translation2d redPose){
      this.bluePose = bluePose;
      this.redPose = redPose;
    }

    public Translation2d getPose(){
      if (isRedAlliance){
      return redPose;
    }else {
      return bluePose
    }

  }
}
}