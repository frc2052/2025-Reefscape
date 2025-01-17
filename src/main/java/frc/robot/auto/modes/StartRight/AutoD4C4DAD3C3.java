package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;

public class AutoD4C4DAD3C3 extends AutoBase {
  //Start Left Equivalent: AutoK4L4DAK3L3

  private final static PathPlannerPath startingPath = Paths.SR_D4;
  
    public AutoD4C4DAD3C3() {
      super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(followPathCommand(startingPath));
    addCommands(followPathCommand(Paths.D4_RL));
    addCommands(followPathCommand(Paths.RL_C4));
    addCommands(followPathCommand(Paths.C4_RL));
    addCommands(followPathCommand(Paths.RL_D3));
    addCommands(followPathCommand(Paths.D3_RL));
    addCommands(followPathCommand(Paths.RL_C3));
  }
}
