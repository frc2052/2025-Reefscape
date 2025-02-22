// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.startCenter;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

/** Add your docs here. */
public class AutoH4LeftAlgaeRemoval extends AutoBase {

  public static final PathPlannerPath startPath = Paths.SC_H4;

  public AutoH4LeftAlgaeRemoval() {
    super(startPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(safeReefAlignment(startPath, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.GH));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(descoreScoreNetAlgae(Paths.GH_SCORE_TO_DESCORE, TargetAction.LA, Paths.GH_NET));
    addCommands(descoreScoreNetAlgae(Paths.NET_IJ, TargetAction.UA, Paths.IJ_NET));
    addCommands(descoreScoreNetAlgae(Paths.NET_KL, TargetAction.LA, Paths.KL_NET));
  }
}
