// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public abstract class AutoBase extends SequentialCommandGroup {
  private final RobotState robotState = RobotState.getInstance();
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private Pose2d startPose;
  private final AutoFactory autoFactory = AutoFactory.getInstance();
  
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

  public void waitTime(){
    addCommands(new WaitCommand(autoFactory.getSavedWaitSeconds()));
  }

  private void setStartPose(Pose2d pathStartPose){
    addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
  }

  protected Command followPathCommand(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  protected static PathPlannerPath getPathFromFile(String pathName){
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return path;
    } catch (Exception e){
      DriverStation.reportError("FAILED TO GET PATH FROM PATHFILE" + e.getMessage(), 
      e.getStackTrace());
      return null;
    }
  }

  // test
  public static Optional<Pose2d> getStartPoseFromAutoFile(String autoName) {
    try {
      List<PathPlannerPath> pathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      return (pathList.get(0).getStartingHolonomicPose());
    } catch (Exception e) {
      DriverStation.reportError(
          "Couldn't get starting pose from auto file: " + autoName + e.getMessage(),
          e.getStackTrace());
      return null;
    }
  }

  // TODO: ----- MANUAL PATH CREATION METHODS ---- //

  public static final class Paths { // to avoid rewriting in every path
    // SL = Start Left 
    // SR = Start Right
    // LL = Left (Barge Side) Coral Station
    // RL = Right (Processor Side) Coral Station
    // Letter + Number = Reef Scoring Position

    // ex:
    // public final static PathPlannerPath AB_BARGECS = getPathFromFile("AB - Barge Coral Station");
    public static final PathPlannerPath test2MeterPath = getPathFromFile("Test Path");
  }
}
