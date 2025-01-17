// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import frc.robot.commands.drive.AlignWithTagCommand;

public class AutoAlignWithTagCommand extends AlignWithTagCommand {
  public AutoAlignWithTagCommand(AlignLocation alignLocation) {
    super(alignLocation, () -> 0, () -> 0, () -> 0);
  }

  @Override
  public void initialize() {
    // PPHolonomicDriveController.overrideRotationFeedback(null);
    // PPHolonomicDriveController.overrideXFeedback(null);
    // PPHolonomicDriveController.overrideYFeedback(null);
  }

  @Override
  public void end(boolean interrupted) {
    // PPHolonomicDriveController.clearFeedbackOverrides();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
