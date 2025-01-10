package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;

public class AutoSRE2D4C4 extends AutoBase {
  // Start Left Equivalent: AutoJ2K4L4

  private static final PathPlannerPath startingPath = Paths.SR_E2;

  public AutoSRE2D4C4() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(followPathCommand(startingPath));
    addCommands(followPathCommand(Paths.E2_RL));
    addCommands(followPathCommand(Paths.RL_D4));
    addCommands(followPathCommand(Paths.D4_RL));
    addCommands(followPathCommand(Paths.RL_C4));
  }
}
