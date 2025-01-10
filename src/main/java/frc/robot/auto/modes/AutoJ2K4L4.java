package frc.robot.auto.modes;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.common.AutoBase;

public class AutoJ2K4L4 extends AutoBase {

  private final static PathPlannerPath startingPath = Paths.SL_J2;
  
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
