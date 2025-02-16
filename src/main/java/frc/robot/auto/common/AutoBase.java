// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.SnapToLocationAngleCommand;
import frc.robot.commands.drive.alignment.AlignWithFieldElementCommand;
import frc.robot.commands.drive.alignment.AlignWithFieldElementCommand.DesiredElement;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TagTrackerType;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;
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

  protected Command delaySelectedTime() {
    return new WaitCommand(autoFactory.getSavedWaitSeconds());
  }

  protected Command getBumpCommand() {
    if (autoFactory.getBumpNeeded()) {
      return new DefaultDriveCommand(() -> -0.3, () -> 0, () -> 0, () -> false)
          .withDeadline(new WaitCommand(0.5));
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

  protected Command snapToReefAngle(TargetFieldLocation snapLocation) {
    return new SnapToLocationAngleCommand(snapLocation, () -> 0, () -> 0, () -> 0, () -> true);
  }

  protected Command algaeReefSideVisionOrPathAlign(
      PathPlannerPath altAlignPath, TargetFieldLocation snaploc) {
    return new SequentialCommandGroup(followPathCommand(altAlignPath), snapToReefAngle(snaploc))
        .until(
            () ->
                vision
                    .getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM, Meters.of(1.5))
                    .isPresent()) // sees tag, goal pose won't be too far
        .andThen(
            new AlignWithFieldElementCommand(
                DesiredElement.REEF,
                AlignOffset.ALGAE_REEF_LOC,
                () -> 0,
                () -> 0,
                () -> 0,
                () -> true));
  }

  protected Command coralReefSideVisionOrPathAlign(
      AlignOffset offset, PathPlannerPath altAlignPath, TargetFieldLocation snaploc) {
    return new SequentialCommandGroup(followPathCommand(altAlignPath), snapToReefAngle(snaploc))
        .until(
            () ->
                vision
                    .getCameraClosestTarget(TagTrackerType.CORAL_REEF_CAM, Meters.of(1.5))
                    .isPresent()) // sees tag, goal pose won't be too far
        .andThen(
            new AlignWithFieldElementCommand(
                snaploc, offset, () -> 0, () -> 0, () -> 0, () -> true));
  }

  protected Command coralSideVisionOrPathAlign(
      AlignOffset offset, PathPlannerPath altAlignPath, TargetFieldLocation snaploc) {
    return new SequentialCommandGroup(followPathCommand(altAlignPath), snapToReefAngle(snaploc))
        .until(
            () ->
                vision.getCameraClosestTarget(TagTrackerType.REAR_CAM, Meters.of(2.0)).isPresent())
        .andThen(
            new AlignWithFieldElementCommand(
                snaploc, offset, () -> 0, () -> 0, () -> 0, () -> true));
  }

  protected Command processorSideVisionOrPathAlign(
      AlignOffset offset, PathPlannerPath altAlignPath, TargetFieldLocation snaploc) {
    return new SequentialCommandGroup(followPathCommand(altAlignPath), snapToReefAngle(snaploc))
        .until(
            () ->
                vision.getCameraClosestTarget(TagTrackerType.ALGAE_CAM, Meters.of(1.0)).isPresent())
        .andThen(
            new AlignWithFieldElementCommand(
                DesiredElement.PROCESSOR, offset, () -> 0, () -> 0, () -> 0, () -> true));
  }

  protected Command toScoringPositionCommand(
      // ReefScoringPosition scorePos
      ) {
    return null; // scorePos.getCommand();
  }

  protected Command toPosition(TargetAction position) {
    return new SequentialCommandGroup(
        // ElevatorCommandFactory.setElevatorPosition(position),
        new WaitCommand(0.75),
        // ElevatorCommandFactory.setElevatorPosition(TargetAction.TR),
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
