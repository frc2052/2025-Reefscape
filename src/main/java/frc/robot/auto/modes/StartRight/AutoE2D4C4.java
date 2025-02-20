package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

@AutoDescription(description = "21 Point Auto - One L2, Two L4")
public class AutoE2D4C4 extends AutoBase {
  // Start Left Equivalent: AutoJ2K4L4

  private static final PathPlannerPath startingPath = Paths.SR_E2;

  public AutoE2D4C4() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());
    addCommands(getBumpCommand());

    addCommands(safeReefAlignment(startingPath, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.EF));
    addCommands(toPosAndScore(TargetAction.L2));
    addCommands(stationVisionOrPathAlign(Paths.E2_RL, TargetFieldLocation.RCS));
    addCommands(HPIntake());
    addCommands(safeReefAlignment(Paths.RL_D4, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.CD));
    addCommands(toPosAndScore(TargetAction.L4));
    addCommands(stationVisionOrPathAlign(Paths.D4_RL, TargetFieldLocation.RCS));
    addCommands(HPIntake());
    addCommands(safeReefAlignment(Paths.RL_C4, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.CD));
    addCommands(toPosAndScore(TargetAction.L4));
  }
}
