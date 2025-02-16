// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import frc.robot.subsystems.superstructure.SuperstructurePosition.AlignOffset;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TagTrackerType;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ProcessorAlign extends AlignWithFieldElementCommand {

  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private PhotonTrackedTarget camTarget;

  public ProcessorAlign(
      AlignOffset offset,
      DoubleSupplier xVal,
      DoubleSupplier yVal,
      DoubleSupplier rotationVal,
      BooleanSupplier fieldCentric) {
    super(() -> offset, xVal, yVal, rotationVal, fieldCentric);
  }

  public FieldElement getElementBasedOnTagID() {
    Optional<PhotonPipelineResult> tar = vision.getCameraClosestTarget(TagTrackerType.ALGAE_CAM);
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
    }

    return FieldElement.NO_ELEMENT;
  }

  @Override
  public FieldElement getFieldElement() {
    return getElementBasedOnTagID();
  }

  @Override
  public void initialize() {
    VisionSubsystem.getInstance().setPrimaryFocus(TagTrackerType.ALGAE_CAM);
    super.initialize();
  }
}
