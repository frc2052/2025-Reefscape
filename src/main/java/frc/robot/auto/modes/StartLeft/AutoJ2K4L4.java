package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

@AutoDescription(description = "21 Point Auto - One L2, Two L4")
public class AutoJ2K4L4 extends AutoBase {
  // Start Right Equivalent: AutoE2D4C4

  private static final PathPlannerPath startingPath = Paths.SL_J2;

  public AutoJ2K4L4() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());

    addCommands(followPathCommand(startingPath));
    addCommands(toPosAndScore(TargetAction.L2));
    addCommands(stationVisionOrPathAlign(Paths.J2_LL, TargetFieldLocation.LCS));
    addCommands(
        reefVisionOrPathAlign(AlignOffset.LEFT_REEF_LOC, Paths.LL_K4, TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(stationVisionOrPathAlign(Paths.K4_LL, TargetFieldLocation.KL));
    addCommands(
        reefVisionOrPathAlign(AlignOffset.LEFT_REEF_LOC, Paths.LL_K4, TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L4));
  }
}
