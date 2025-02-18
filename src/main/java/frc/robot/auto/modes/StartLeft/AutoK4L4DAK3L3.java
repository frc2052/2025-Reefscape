package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

@AutoDescription(description = "29 Point Auto - Two L4, Remove Algae, Two L3")
public class AutoK4L4DAK3L3 extends AutoBase {
  // Start Right Equivalent: AutoSLK4L4DAK3L3

  private static final PathPlannerPath startingPath = Paths.SL_K4;

  public AutoK4L4DAK3L3() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());

    addCommands(
        reefVisionOrPathAlign(AlignOffset.LEFT_REEF_LOC, startingPath, TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L4));

    addCommands(stationVisionOrPathAlign(Paths.K4_LL, TargetFieldLocation.KL));
    addCommands(
        reefVisionOrPathAlign(
            AlignOffset.RIGHT_CORAL_STATION_LOC, Paths.LL_L4, TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(
        reefVisionOrPathAlign(AlignOffset.LEFT_REEF_LOC, Paths.LL_K3, TargetFieldLocation.KL));
    // addCommands(followPathCommand(Paths.LL_K3));
    // addCommands(reefSideVisionOrPathAlign(AlignOffset.LEFT, Paths.LL_K3, SnapLocation.ReefKL));
    addCommands(toPosition(TargetAction.L3));
    addCommands(followPathCommand(Paths.K3_LL));
    addCommands(followPathCommand(Paths.LL_L3));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.RIGHT, Paths.LL_L3,
    // SnapLocation.ReefKL));
    addCommands(toPosAndScore(TargetAction.L3));
  }
}
