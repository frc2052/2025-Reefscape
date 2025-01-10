package frc.robot.auto.modes;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.common.AutoBase;

public class AutoK4L4DAK3L3 extends AutoBase {

  private final static PathPlannerPath startingPath = Paths.SL_K4;
  
    public AutoK4L4DAK3L3() {
      super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(followPathCommand(startingPath));
    addCommands(followPathCommand(Paths.K4_LL));
    addCommands(followPathCommand(Paths.LL_L4));
    addCommands(followPathCommand(Paths.L4_LL));
    addCommands(followPathCommand(Paths.LL_K3));
    addCommands(followPathCommand(Paths.K3_LL));
    addCommands(followPathCommand(Paths.LL_L3));
  }
}
