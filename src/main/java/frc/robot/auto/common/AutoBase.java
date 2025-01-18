// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.AlignWithTagCommand;
import frc.robot.commands.drive.AlignWithTagCommand.AlignLocation;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public abstract class AutoBase extends SequentialCommandGroup {
  private final RobotState robotState = RobotState.getInstance();
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final AutoFactory autoFactory = AutoFactory.getInstance();
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

  public void delaySelectedTime() {
    addCommands(new WaitCommand(autoFactory.getSavedWaitSeconds()));
  }

  private void setStartPose(Pose2d pathStartPose) {
    addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
  }

  protected Command followPathCommand(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  // protected Command alignToReefAngle(SnapLocation snapLocation, PathPlannerPath path) {
  //   return new SnapToLocationAngleCommand(snapLocation, () -> 0, () -> 0, () -> 0, () -> true);
  //   PPHolonomicDriveController.overrideXFeedback(null);
  // }

  protected Command alignWithReefSideCommand(
      AlignLocation alignLocation, PathPlannerPath altAlignPath) {
    return new ConditionalCommand(
        new AlignWithTagCommand(alignLocation, () -> 0, () -> 0, () -> 0),
        followPathCommand(altAlignPath),
        () -> vision.getReefCamClosestTarget().isPresent());
    // return new SnapToLocationAngleCommand(snapLocation, () -> 0, () -> 0, () -> 0, () -> true);
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

  public static final class Paths { // to avoid rewriting in every path

    // SL = Start Left
    // SR = Start Right
    // LL = Left (Barge Side) Coral Station
    // RL = Right (Processor Side) Coral Station
    // DA = Descore (Remove) Algae From Reef
    // Letter + Number = Reef Scoring Position

    // ex:
    // public final static PathPlannerPath AB_BARGECS = getPathFromFile("AB - Barge Coral Station");
    public static final PathPlannerPath TEST_PATH_SL_EF = getPathFromFile("Test Auto - SL-EF");

    public static final PathPlannerPath J2_LL = getPathFromFile("J2 LL");
    public static final PathPlannerPath K3_LL = getPathFromFile("K3 LL");
    public static final PathPlannerPath K4_LL = getPathFromFile("K4 LL");
    public static final PathPlannerPath L4_LL = getPathFromFile("L4 LL");
    public static final PathPlannerPath LL_K3 = getPathFromFile("LL K3");
    public static final PathPlannerPath LL_K4 = getPathFromFile("LL K4");
    public static final PathPlannerPath LL_L3 = getPathFromFile("LL L3");
    public static final PathPlannerPath LL_L4 = getPathFromFile("LL L4");
    public static final PathPlannerPath SL_J2 = getPathFromFile("LL J2");
    public static final PathPlannerPath SL_K4 = getPathFromFile("LL K4");
    public static final PathPlannerPath SR_E2 = getPathFromFile("SR E2");
    public static final PathPlannerPath E2_RL = getPathFromFile("E2 RL");
    public static final PathPlannerPath RL_D4 = getPathFromFile("RL D4");
    public static final PathPlannerPath D4_RL = getPathFromFile("D4 RL");
    public static final PathPlannerPath RL_C4 = getPathFromFile("RL C4");
    public static final PathPlannerPath C4_RL = getPathFromFile("C4 RL");
    public static final PathPlannerPath SR_D4 = getPathFromFile("SR D4");
    public static final PathPlannerPath RL_D3 = getPathFromFile("RL D3");
    public static final PathPlannerPath D3_RL = getPathFromFile("D3 RL");
    public static final PathPlannerPath RL_C3 = getPathFromFile("RL C3");
  }
}
