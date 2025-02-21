package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

@AutoDescription(description = "29 Point Auto - Two L4, Remove Algae, Two L3")
public class AutoD4C4DAD3C3 extends AutoBase {
  // Start Left Equivalent: AutoK4L4DAK3L3

  private static final PathPlannerPath startingPath = Paths.SR_D4;

  public AutoD4C4DAD3C3() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());
    addCommands(getBumpCommand());

    addCommands(
        safeReefAlignment(startingPath, AlignOffset.RIGHT_REEF_LOC, TargetFieldLocation.CD));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(safeStationAlignment(Paths.D4_RL));
    addCommands(safeReefAlignment(Paths.RL_C4, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(
        descoreScoreNetAlgae(
            Paths.CD_SCORE_TO_DESCORE, TargetAction.UA, Paths.CD_NET)); // CD is upper algae
    addCommands(safeStationAlignment(Paths.NET_SCORE_RIGHT_STATION));
    addCommands(HPIntake());
    addCommands(safeReefAlignment(Paths.RL_D3, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.CD));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(safeStationAlignment(Paths.D3_RL));
    addCommands(safeReefAlignment(Paths.RL_C3, AlignOffset.RIGHT_REEF_LOC, TargetFieldLocation.CD));
  }
}
