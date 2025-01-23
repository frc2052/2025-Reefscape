// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import frc.robot.commands.drive.SnapToLocationAngleCommand;

public class AutoSnapToLocationAngleCommand extends SnapToLocationAngleCommand {
  public AutoSnapToLocationAngleCommand(SnapLocation snapLocation) {
    super(snapLocation, () -> 0, () -> 0, () -> 0, () -> false);
  }

  @Override
  public void initialize() {
    // no overrides necessary? just a regular command
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
