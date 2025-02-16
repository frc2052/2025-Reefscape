// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import frc.robot.subsystems.superstructure.SuperstructurePosition.AlignOffset;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.TagTrackerType;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlgaeReefAlign extends AlignWithFieldElementCommand {

  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private PhotonTrackedTarget camTarget = new PhotonTrackedTarget();
  private final boolean exclusive;

  public AlgaeReefAlign(
      boolean exclusive,
      DoubleSupplier xVal,
      DoubleSupplier yVal,
      DoubleSupplier rotationVal,
      BooleanSupplier fieldCentric) {
    super(() -> AlignOffset.ALGAE_REEF_LOC, xVal, yVal, rotationVal, fieldCentric);
    this.exclusive = exclusive;
  }

  public FieldElement getElementBasedOnTagID() {
    Optional<PhotonPipelineResult> tar = vision.getCameraClosestTarget(TagTrackerType.ALGAE_CAM);
    if (tar.isPresent()) {
      camTarget = tar.get().getBestTarget();

      System.out.println(this.getName() + " COMMAND SEES TAG: " + camTarget.fiducialId);

      if (camTarget.fiducialId >= 17 && camTarget.fiducialId <= 22) {
        return FieldElement.BLUE_REEF;
      } else if (camTarget.fiducialId >= 6 && camTarget.fiducialId <= 11) {
        return FieldElement.RED_REEF;
      }
    }

    return FieldElement.NO_ELEMENT;
  }

  @Override
  public FieldElement getFieldElement() {
    return getElementBasedOnTagID();
  }

  @Override
  protected int getSpecificReefSideTag() {
    int goalTagID = SuperstructureSubsystem.getInstance().getTargetReefSide().getTagID();
    if (exclusive && camTarget.getFiducialId() != goalTagID) {
      return -1; // don't align, not correct tag
    } else {
      return 0; // good to align
    }
  }

  @Override
  public void initialize() {
    VisionSubsystem.getInstance().setPrimaryFocus(TagTrackerType.ALGAE_CAM);
    super.initialize();
  }
}
