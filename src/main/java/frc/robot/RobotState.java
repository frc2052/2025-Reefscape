package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private static FieldLocation loca;

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

  public void setFieldLocation(FieldLocation location){
      loca = location;
    }
  public static FieldLocation getFieldLocation(){
    return loca;
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

  public enum FieldLocation {
    HP(new Translation2d(0,0)),// adjust these values. 
    REEF(new Translation2d(0,0)),
    BARGE(new Translation2d(0,0)),
    PROCESSOR(new Translation2d(0,0)),
    TRAVEL (new Translation2d(0,0));

    public final Translation2d pose; 

    private FieldLocation(Translation2d pose){
      this.pose = pose;
    }

    public Translation2d getPose(){
     return pose;

  }
}
}