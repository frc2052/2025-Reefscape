// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team2052.lib.planners.AutoAlignPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.AimingCalculator.ElementFieldPosition;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AlignWithFieldElementCommand extends DefaultDriveCommand {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final RobotState robotState = RobotState.getInstance();

  private Pose2d goalPose;
  private Supplier<AllAlignOffsets> scoringLocation;
  private AutoAlignPlanner planner;

  private SwerveRequest.ApplyFieldSpeeds drive =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  public AlignWithFieldElementCommand(
      Supplier<AllAlignOffsets> scoringLocation,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldCentric) {
    super(xSupplier, ySupplier, rotationSupplier, fieldCentric);

    this.scoringLocation = scoringLocation;
    planner = new AutoAlignPlanner();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    goalPose = null;
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
    setGoalPose(getFieldElement());
    super.execute();
  }

  public FieldElement getFieldElement() { // if not overriden, defaults to NO_ELEMENT
    return FieldElement.NO_ELEMENT;
  }

  // field element is passed in from individual alignment clases
  // either pass no_element (no target) or field element
  public void setGoalPose(FieldElement element) { // overriden in individual classes

    Logger.recordOutput("Target for Alignment: " + element.getDisplayName(), true);

    // individual commands did NOT see tag
    if (element.equals(FieldElement.NO_ELEMENT)) {
      goalPose = null;
    }

    // individual commands saw tag that IS NOT a reeef tag
    else if (!FieldElement.checkIsReef(element)) { // we passed in element: NOT red or blue reef
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(element.getTagID());
      goalPose =
          AimingCalculator.horizontalAjustment(
              Meters.of(scoringLocation.get().transform.getY()),
              AimingCalculator.scaleFromReef(
                  tagPose.get().toPose2d(),
                  Meters.of(scoringLocation.get().transform.getX()),
                  robotState.isRedAlliance()));
      Logger.recordOutput("Tag Pose Present", true);
    } else { // TODO: set up reef target for alignment intead of old command
      Logger.recordOutput("Reef Target for Alignment: " + element.getDisplayName(), true);

      if (getSpecificReefSideTag() != 0) { // only 0 from class default and specific command default
        goalPose = null;

      } else {
        Optional<Pose3d> tagPose =
            VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(getSpecificReefSideTag());
        if (tagPose.isPresent()) {
          goalPose =
              AimingCalculator.horizontalAjustment(
                  Meters.of(scoringLocation.get().transform.getY()),
                  AimingCalculator.scaleAll(
                      goalPose, Meters.of(scoringLocation.get().transform.getX()), element));
        } else {
          Logger.recordOutput("Reef Tag Pose Present", false);
          goalPose = null;
        }
      }
    }
  }

  public int getSpecificReefSideTag() {
    return 0;
  }

  @Override
  public boolean isFinished() {
    if (planner.getAutoAlignComplete()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interuppted) {
    planner.resetPlanner();
  }

  // ---------------------- ENUMS ---------------------- //

  public static enum FieldElement {
    RED_PROCESSOR("RED Processor", ElementFieldPosition.RED_PROCESSOR.getElemPose(), 3),
    BLUE_PROCESSOR("BLUE Processor", ElementFieldPosition.BLUE_PROCESSOR.getElemPose(), 16),
    RED_REEF("RED Reef Side", ElementFieldPosition.RED_REEF.getElemPose()),
    BLUE_REEF("BLUE Reef Side", ElementFieldPosition.BLUE_REEF.getElemPose()),
    RED_L_CORALSTATION(
        "RED LEFT Coral Station", ElementFieldPosition.REDSIDE_LEFT_STATION.getElemPose(), 1),
    RED_R_CORALSTATION(
        "RED RIGHT Coral Station", ElementFieldPosition.REDSIDE_RIGHT_STAITON.getElemPose(), 2),
    BLUE_L_CORALSTATION(
        "BLUE LEFT Coral Station", ElementFieldPosition.BLUESIDE_LEFT_STATION.getElemPose(), 12),
    BLUE_R_CORALSTATION(
        "BLUE RIGHT Coral Station", ElementFieldPosition.BLUESIDE_RIGHT_STATION.getElemPose(), 13),
    NO_ELEMENT("You Have No Element", null, 0);

    public String elemName;
    public Translation2d elemFieldPosition;
    public int fieldTagID;

    public String getDisplayName() {
      return elemName;
    }

    public Translation2d getFieldPosition() {
      return elemFieldPosition;
    }

    public int getTagID() {
      return fieldTagID;
    }

    private FieldElement(String name, Translation2d fieldPos, int tagID) {
      elemName = name;
      elemFieldPosition = fieldPos;
      fieldTagID = tagID;
    }

    private FieldElement(String name, Translation2d fieldPos) {
      elemName = name;
      elemFieldPosition = fieldPos;
      fieldTagID = 0;
    }

    public static boolean checkIsReef(FieldElement element) {
      if (element.equals(FieldElement.RED_REEF) || element.equals(FieldElement.BLUE_REEF)) {
        return true;
      }
      return false;
    }

    public static boolean checkIsProcessor(FieldElement element) {
      if (element.equals(FieldElement.RED_PROCESSOR)
          || element.equals(FieldElement.BLUE_PROCESSOR)) {
        return true;
      }
      return false;
    }

    public static boolean checkIsCoralStation(FieldElement element) {
      if (element.equals(FieldElement.RED_L_CORALSTATION)
          || element.equals(FieldElement.RED_R_CORALSTATION)
          || element.equals(FieldElement.BLUE_L_CORALSTATION)
          || element.equals(FieldElement.BLUE_R_CORALSTATION)) {
        return true;
      }
      return false;
    }
  }

  public enum AllAlignOffsets { // TODO verify offsets
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
