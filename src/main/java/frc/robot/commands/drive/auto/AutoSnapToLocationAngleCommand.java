// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.commands.drive.SnapToLocationAngleCommand;

public class AutoSnapToLocationAngleCommand extends SnapToLocationAngleCommand {
  public AutoSnapToLocationAngleCommand(SnapLocation snapLocation) {
    super(snapLocation, () -> 0, () -> 0, () -> 0, () -> false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // PPHolonomicDriveController.overrideRotationFeedback(null);
    // PPHolonomicDriveController.overrideXFeedback(null);
    // PPHolonomicDriveController.overrideYFeedback(null);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // PPHolonomicDriveController.clearFeedbackOverrides();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
