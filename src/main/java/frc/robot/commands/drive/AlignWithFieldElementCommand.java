// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team2052.lib.planners.AutoAlignPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.AimingCalculator.ElementFieldPosition;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class AlignWithFieldElementCommand extends DefaultDriveCommand {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final RobotState robotState = RobotState.getInstance();

  private PhotonTrackedTarget camTarget;
  private Pose2d goalPose;

  private Supplier<AllAlignOffsets> scoringLocation;
  private FieldElement fieldElement;

  private AutoAlignPlanner planner;

  private SwerveRequest.ApplyFieldSpeeds drive =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithFieldElementCommand( // TODO: what field element it is depends on the tag we see: how to use in teleop?
      Supplier<FieldElement> fieldEl, // what object
      Supplier<AllAlignOffsets> scoringLocation,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);

    System.out.println("ALIGN WITH FIELD ELEMENT COMMAND CALLED FOR: " + fieldElement.getDisplayName());

    this.scoringLocation = scoringLocation;
    this.fieldElement = fieldEl.get();
    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    goalPose = null;
    if (fieldElement.equals(FieldElement.RED_PROCESSOR) || fieldElement.equals(FieldElement.BLUE_PROCESSOR)) {
      robotState.setReefTracking(false);
      robotState.setProcessorTracking(true);
      robotState.setStationTracking(false);
    } else if (fieldElement.equals(FieldElement.RED_REEF)
        || fieldElement.equals(FieldElement.BLUE_REEF)) {
      robotState.setReefTracking(true);
      robotState.setProcessorTracking(false);
      robotState.setStationTracking(false);
    } else { // align coral station
      robotState.setReefTracking(false);
      robotState.setProcessorTracking(false);
      robotState.setStationTracking(true);
    }
  }

  @Override
  public SwerveRequest getSwerveRequest() {
    if (goalPose != null) {
      return drive.withSpeeds(planner.calculate(robotState.getFieldToRobot(), goalPose));
    } else {
      return super.getSwerveRequest();
    }
  }

  @Override
  public void execute() { 
    switch (fieldElement) {
      case RED_PROCESSOR:
        setGoalPoseProcessor();
        break;
      case RED_REEF:
        setGoalPoseReef(true);
        break;
      case BLUE_REEF:
        setGoalPoseReef(false);
        break;
      case BLUE_L_CORALSTATION:
        setGoalPoseCoralStation(false, false);
        break;
      case BLUE_R_CORALSTATION:
        setGoalPoseCoralStation(false, true);
        break;
      case RED_L_CORALSTATION:
        setGoalPoseCoralStation(true, false);
        break;
      case RED_R_CORALSTATION:
        setGoalPoseCoralStation(true, false);
        break;
      default:
        break;
    }
    super.execute();
  }

  public void setGoalPoseReef(boolean isRedAlliance) {
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      Logger.recordOutput("Target for Alignment: " + fieldElement.getDisplayName(), true);
      camTarget = tar.get().getBestTarget();
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(camTarget.fiducialId);
      if (tagPose.isPresent()) {
        goalPose =
            AimingCalculator.horizontalAjustment(
                Meters.of(scoringLocation.get().transform.getY()),
                AimingCalculator.scaleFromReef(
                    tagPose.get().toPose2d(),
                    Meters.of(scoringLocation.get().transform.getX()),
                    robotState.isRedAlliance()));
        Logger.recordOutput("Tag Pose Present", true);
      } else {
        Logger.recordOutput("Tag Pose Present", false);
        goalPose = null;
      }
    } else {
      Logger.recordOutput("Target for Alignment: " + fieldElement.getDisplayName(), false);
      camTarget = null;
    }
  }  
  
  public void setGoalPoseProcessor() {
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      Logger.recordOutput("Target for Alignment: " + fieldElement.getDisplayName(), true);
      camTarget = tar.get().getBestTarget();
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(camTarget.fiducialId);

      if (tagPose.isPresent()) {
        goalPose =
            AimingCalculator.horizontalAjustment(
                Meters.of(scoringLocation.get().transform.getY()),
                AimingCalculator.scaleFromProcessor(
                    tagPose.get().toPose2d(),
                    Meters.of(scoringLocation.get().transform.getX()),
                    robotState.isRedAlliance()));
        Logger.recordOutput("Tag Pose Present", true);
      } else {
        Logger.recordOutput("Tag Pose Present", false);
        goalPose = null;
      }
    } else {
      Logger.recordOutput("Target for Alignment: " + fieldElement.getDisplayName(), false);
      camTarget = null;
    }
  }

  public boolean getStationSideRight(int id) { // TODO: how do we check we aren't seeing either
    if (id == 1 || id == 13) { // 1 = red left, 13 = blue left
      return false;
    } else { // 2 = red right, 12 = blue right
      return true;
    }
  }

  public void setGoalPoseCoralStation(boolean isRedAlliance, boolean rightSide) {
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      Logger.recordOutput("Target for Alignment: " + fieldElement.getDisplayName(), true);
      camTarget = tar.get().getBestTarget();
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(camTarget.fiducialId);

      if (tagPose.isPresent()) {
        goalPose =
            AimingCalculator.horizontalAjustment(
                Meters.of(scoringLocation.get().transform.getY()),
                AimingCalculator.scaleFromCoralStation(
                    tagPose.get().toPose2d(),
                    Meters.of(scoringLocation.get().transform.getX()),
                    robotState.isRedAlliance(),
                    getStationSideRight(camTarget.fiducialId)));
        Logger.recordOutput("Tag Pose Present", true);
      } else {
        Logger.recordOutput("Tag Pose Present", false);
        goalPose = null;
      }
    } else {
      Logger.recordOutput("Target for Alignment: " + fieldElement.getDisplayName(), false);
      camTarget = null;
    }
  }

  public static enum FieldElement {
    RED_PROCESSOR("RED Processor", ElementFieldPosition.RED_PROCESSOR.getElemPose()),
    BLUE_PROCESSOR("BLUE Processor", ElementFieldPosition.BLUE_PROCESSOR.getElemPose()),
    RED_REEF("RED Reef Side", ElementFieldPosition.RED_REEF.getElemPose()),
    BLUE_REEF("BLUE Reef Side", ElementFieldPosition.BLUE_REEF.getElemPose()),
    RED_L_CORALSTATION("RED LEFT Coral Station", ElementFieldPosition.REDSIDE_LEFT_STATION.getElemPose()),
    RED_R_CORALSTATION("RED RIGHT Coral Station", ElementFieldPosition.REDSIDE_RIGHT_STAITON.getElemPose()),
    BLUE_L_CORALSTATION("BLUE LEFT Coral Station", ElementFieldPosition.BLUESIDE_LEFT_STATION.getElemPose()), 
    BLUE_R_CORALSTATION("BLUE RIGHT Coral Station", ElementFieldPosition.BLUESIDE_RIGHT_STATION.getElemPose());

    public String elemName;
    public Translation2d elemFieldPosition; 

    public String getDisplayName() {
      return elemName;
    }

    public Translation2d getFieldPosition(){
        return elemFieldPosition;
    }

    private FieldElement(String name, Translation2d fieldPos) {
      elemName = name;
      elemFieldPosition = fieldPos;
    }
  }

  public enum AllAlignOffsets { // provides an offset from the april tag
    LEFT_REEF_LOC(new Transform2d(0.5, 0.25, new Rotation2d(0))),
    MIDDLE_REEF_LOC(new Transform2d(0.5, 0.0, new Rotation2d(0))),
    RIGHT_REEF_LOC(new Transform2d(0.5, -0.25, new Rotation2d(0))),

    PROCESSOR_MIDDLE_LOC(new Transform2d(0.5, -0.25, new Rotation2d(0))),

    LEFT_CORAL_LOC(new Transform2d(0.5, -0.25, new Rotation2d(0))),
    RIGHT_CORAL_LOC(new Transform2d(0.5, -0.25, new Rotation2d(0)));

    public Transform2d transform;

    private AllAlignOffsets(Transform2d gt) {
      this.transform = gt;
    }
  }
}
