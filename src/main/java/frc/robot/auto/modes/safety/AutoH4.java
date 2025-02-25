// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.safety;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;

/** Add your docs here. */
public class AutoH4 extends AutoBase {

  public static PathPlannerPath startPath = Paths.SC_H4;

  public AutoH4() {
    super(startPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(getBumpCommand());
    addCommands(delaySelectedTime());

    addCommands(followPathCommand(startPath));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(elevatorToPos(TargetAction.L1H));
  }
}
