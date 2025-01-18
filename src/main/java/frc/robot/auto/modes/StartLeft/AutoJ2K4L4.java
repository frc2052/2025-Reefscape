package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;

public class AutoJ2K4L4 extends AutoBase {
  // Start Right Equivalent: AutoE2D4C4

  private static final PathPlannerPath startingPath = Paths.SL_J2;

  public AutoJ2K4L4() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(followPathCommand(startingPath));
    addCommands(followPathCommand(Paths.J2_LL));
    addCommands(followPathCommand(Paths.LL_K4));
    addCommands(followPathCommand(Paths.K4_LL));
    addCommands(followPathCommand(Paths.LL_L4));
  }
}
