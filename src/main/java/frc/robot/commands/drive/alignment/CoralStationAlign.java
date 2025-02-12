// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.alignment;

import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralStationAlign extends AlignWithFieldElementCommand {

  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private PhotonTrackedTarget camTarget;
  private final RobotState robotState = RobotState.getInstance();

  public CoralStationAlign(
      AllAlignOffsets offset,
      Supplier<Double> xVal,
      Supplier<Double> yVal,
      Supplier<Double> rotationVal,
      Supplier<Boolean> fieldCentric) {
    super(() -> offset, () -> xVal.get(), () -> yVal.get(), () -> rotationVal.get(), () -> fieldCentric.get());
  }

  public FieldElement getElementBasedOnTagID() { // TODO: print what tag we are looking at
    Optional<PhotonPipelineResult> tar = vision.getCoralStationTarget();
    if (tar.isPresent()) {
      camTarget = tar.get().getBestTarget();

      System.out.println(this.getName() + " COMMAND SEES TAG: " + camTarget.fiducialId);

      if (camTarget.fiducialId == 1) {
        return FieldElement.RED_L_CORALSTATION;
      } else if (camTarget.fiducialId == 2) {
        return FieldElement.RED_R_CORALSTATION;
      } else if (camTarget.fiducialId == 12) {
        return FieldElement.BLUE_R_CORALSTATION;
      } else if (camTarget.fiducialId == 13) {
        return FieldElement.BLUE_L_CORALSTATION;
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
    robotState.setProcessorTracking(false);
    robotState.setStationTracking(true);
  }

  @Override
  public void execute() {
    super.execute();
  }
}
