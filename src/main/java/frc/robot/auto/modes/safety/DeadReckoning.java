// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.safety;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.drive.DefaultDriveCommand;

/** Add your docs here. */
public class DeadReckoning extends AutoBase {

  private static final PathPlannerPath holderPath = Paths.SL_J2;

  public DeadReckoning() {
    super(holderPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(manualZero());
    addCommands(
        new DefaultDriveCommand(() -> 0.5, () -> 0.0, () -> 0.0, () -> false).withTimeout(2.0));
  }
}
