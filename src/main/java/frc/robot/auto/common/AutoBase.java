// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public abstract class AutoBase extends SequentialCommandGroup {
  private final RobotState robotState = RobotState.getInstance();
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private Pose2d startPose;

  protected AutoBase(Optional<Pose2d> pathStartPose) {
    if (pathStartPose.isEmpty()) {
      startPose = new Pose2d();
    } else {
      startPose = pathStartPose.get();
    }

    if (RobotState.getInstance().isRedAlliance()) {
      startPose = FlippingUtil.flipFieldPose(startPose);
    }

    setStartPose(startPose);
    Logger.recordOutput("Red Alliance", this.robotState.isRedAlliance());
    Logger.recordOutput("Auto Starting Pose", startPose);
  }

  public abstract void init(); // defined in each Auto class

  private void setStartPose(Pose2d pathStartPose) {
    addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
  }

  protected Command followPathCommand(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  protected static PathPlannerPath getPathFromFile(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return path;
    } catch (Exception e) {
      DriverStation.reportError(
          "FAILED TO GET PATH FROM PATHFILE" + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public static final class Paths { // to avoid rewriting in every path

    // SL = Start Left
    // SR = Start Right
    // LL = Left (Barge Side) Coral Station
    // RL = Right (Processor Side) Coral Station
    // Letter + Number = Reef Scoring Position

    // ex:
    // public final static PathPlannerPath AB_BARGECS = getPathFromFile("AB - Barge Coral Station");

  }
}
