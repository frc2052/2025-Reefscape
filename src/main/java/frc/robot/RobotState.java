package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private AlignOffset selectedAlignOffset = AlignOffset.MIDDLE_REEF_LOC;

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

  public void addDrivetrainState(SwerveDriveState drivetrainState) {
    this.drivetrainState = drivetrainState;
  }

  public void setAlignOffset(AlignOffset offset) {
    selectedAlignOffset = offset;
  }

  public AlignOffset getAlignOffset() {
    return selectedAlignOffset;
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
