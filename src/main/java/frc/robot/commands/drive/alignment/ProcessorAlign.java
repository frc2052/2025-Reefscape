// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ProcessorAlign extends AlignWithFieldElementCommand {

  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private PhotonTrackedTarget camTarget;
  private final RobotState robotState = RobotState.getInstance();

  public ProcessorAlign(
      AllAlignOffsets offset,
      DoubleSupplier xVal,
      DoubleSupplier yVal,
      DoubleSupplier rotationVal,
      BooleanSupplier fieldCentric) {
    super(() -> offset, xVal, yVal, rotationVal, fieldCentric);
  }

public FieldElement getElementBasedOnTagID() {
    Optional<PhotonPipelineResult> tar = vision.getCoralStationTarget();
    if (tar.isPresent()) {
      camTarget = tar.get().getBestTarget();

      System.out.println(this.getName() + " COMMAND SEES TAG: " + camTarget.fiducialId);

      if (camTarget.fiducialId == 3) {
        return FieldElement.RED_PROCESSOR;
      } else if (camTarget.fiducialId == 16) {
        return FieldElement.BLUE_PROCESSOR;
      } else {
        return FieldElement.NO_ELEMENT;
      }
    } else {
      camTarget = null;
      return FieldElement.NO_ELEMENT;
    }
    }

  @Override
  public FieldElement getFieldElement() {
    return getElementBasedOnTagID();
  }

  @Override
  public void initialize() {
    robotState.setReefTracking(false);
    robotState.setProcessorTracking(true);
    robotState.setStationTracking(false);
  }

  @Override
  public void execute() {
    super.execute();
  }
}
