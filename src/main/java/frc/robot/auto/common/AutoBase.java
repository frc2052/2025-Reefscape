// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.commands.drive.AlignWithReefCommand;
import frc.robot.commands.drive.AlignWithReefCommand.AlignLocation;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.SnapToLocationAngleCommand;
import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocation;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.commands.superstructure.ReefScoringCommandFactory.ReefScoringPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

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

  protected Command delaySelectedTime() {
    return new WaitCommand(autoFactory.getSavedWaitSeconds());
  }

  protected Command getBumpCommand() {
    if (autoFactory.getBumpNeeded()) {
      return new DefaultDriveCommand(() -> -0.5, () -> 0, () -> 0, () -> false)
          .withDeadline(new WaitCommand(1));
    } else {
      return new InstantCommand();
    }
  }

  private void setStartPose(Pose2d pathStartPose) {
    addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
  }

  protected Command followPathCommand(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  protected Command snapToReefAngle(SnapLocation snapLocation) {
    return new SnapToLocationAngleCommand(snapLocation, () -> 0, () -> 0, () -> 0, () -> true);
  }

  protected Command reefSideVisionOrPathAlign(
      AlignLocation alignLocation, PathPlannerPath altAlignPath, SnapLocation snaploc) {
    return new SequentialCommandGroup(followPathCommand(altAlignPath), snapToReefAngle(snaploc))
        .until(
            () ->
                vision.getReefCamClosestTarget().isPresent()
                    && getDistanceToGoal(() -> alignLocation)
                        < 3.0) // sees tag, goal pose won't be too far
        .andThen(
            new AlignWithReefCommand(() -> alignLocation, () -> 0, () -> 0, () -> 0, () -> true));
  }

  public double getDistanceToGoal(Supplier<AlignLocation> scoringLoc) {
    Pose2d goalPose;
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      System.out.println("SEES TARGET IN AUTOS");
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(tar.get().getBestTarget().fiducialId);
      if (tagPose.isPresent()) {
        goalPose = tagPose.get().toPose2d().transformBy(scoringLoc.get().transform);
      } else {
        return 50; // any number greater than our bound to switch to vision
      }
    } else {
      return 50;
    }

    Translation2d goalTranslation = new Translation2d(goalPose.getX(), goalPose.getY());
    return goalTranslation.getDistance(
        new Translation2d(
            RobotState.getInstance().getFieldToRobot().getX(),
            RobotState.getInstance().getFieldToRobot().getY()));
  }

  protected Command toScoringPositionCommand(ReefScoringPosition scorePos) {
    return scorePos.getCommand();
  }

  protected Command toPosition(ElevatorPosition position) {
    return new SequentialCommandGroup(
        ElevatorCommandFactory.setElevatorPosition(position),
        new WaitCommand(0.75),
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.TRAVEL),
        new WaitCommand(0.5));
  }

  protected static PathPlannerPath getPathFromFile(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return path;
    } catch (Exception e) {
      DriverStation.reportError(
          "FAILED TO GET PATH FROM PATHFILE" + pathName + e.getMessage(), e.getStackTrace());
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
    // public static final PathPlannerPath TEST_PATH_SL_EF = getPathFromFile("Test Auto - SL-EF");

    // each letter to each loading station

    public static final PathPlannerPath LL_STOP = getPathFromFile("LL STOP");

    // CD
    public static final PathPlannerPath RL_C4 = getPathFromFile("RL C");
    public static final PathPlannerPath C4_RL = getPathFromFile("C RL");
    public static final PathPlannerPath RL_C3 = getPathFromFile("RL C");
    public static final PathPlannerPath RL_D4 = getPathFromFile("RL D");
    public static final PathPlannerPath D4_RL = getPathFromFile("D RL");
    public static final PathPlannerPath SR_D4 = getPathFromFile("SR D");
    public static final PathPlannerPath RL_D3 = getPathFromFile("RL D");
    public static final PathPlannerPath D3_RL = getPathFromFile("D RL");

    // ef
    public static final PathPlannerPath E2_RL = getPathFromFile("E RL");
    public static final PathPlannerPath SR_E2 = getPathFromFile("SR E");
    // public static final PathPlannerPath PROCESS_EF = getPathFromFile("Processor EF");
    // public static final PathPlannerPath EF_PROCESS = getPathFromFile("EF Processor");

    // gh
    public static final PathPlannerPath SC_H4 = getPathFromFile("SC H");
    public static final PathPlannerPath H4_PROCESS = getPathFromFile("H Processor");

    // ij
    public static final PathPlannerPath J2_LL = getPathFromFile("J LL");
    public static final PathPlannerPath SL_J2 = getPathFromFile("SL J");

    // kl
    public static final PathPlannerPath K3_LL = getPathFromFile("K LL");
    public static final PathPlannerPath K4_LL = getPathFromFile("K LL");
    public static final PathPlannerPath L4_LL = getPathFromFile("L LL");
    public static final PathPlannerPath LL_K3 = getPathFromFile("LL K");
    public static final PathPlannerPath LL_K4 = getPathFromFile("LL K");
    public static final PathPlannerPath LL_L3 = getPathFromFile("LL L");
    public static final PathPlannerPath LL_L4 = getPathFromFile("LL L");
    public static final PathPlannerPath SL_K4 = getPathFromFile("SL K");
  }
}
