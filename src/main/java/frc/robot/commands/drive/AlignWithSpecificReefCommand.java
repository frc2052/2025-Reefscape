// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team2052.lib.planners.AutoAlignPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AimingCalculator;

public class AlignWithSpecificReefCommand extends AlignWithReefCommand {
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final int tagID;
  private final RobotState robotState = RobotState.getInstance();


  private PhotonTrackedTarget target;

  /** Creates a new AlignWithSpecificReefCommand. */
  public AlignWithSpecificReefCommand(
    Supplier<AlignLocation> scoringLocation,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier rotationSupplier,
    BooleanSupplier fieldCentric,
    int tagID
  ) {
    super(scoringLocation, xSupplier, ySupplier, rotationSupplier, fieldCentric);
    this.tagID = tagID;
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    Pose2d goalPose;
    if (tar.isPresent() && tar.get().getBestTarget().getFiducialId() == tagID) {
      target = tar.get().getBestTarget();
      Optional<Pose3d> tagPose =
          VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(target.fiducialId);
      if (tagPose.isPresent()) {
        goalPose =
            AimingCalculator.horizontalAjustment(
                Meters.of(super.getScoringLocation().transform.getY()),
                AimingCalculator.scaleFromReef(
                    tagPose.get().toPose2d(),
                    Meters.of(super.getScoringLocation().transform.getX()),
                    robotState.isRedAlliance()));
        super.setGoal(goalPose);
        Logger.recordOutput("Target for Alignment", true);
      } else {
        goalPose = null;
      }
    } else {
      Logger.recordOutput("Target for Alignment", false);
      goalPose = null;
      target = null;
    }
    drivetrain.setControl(getSwerveRequest());
  }

}
