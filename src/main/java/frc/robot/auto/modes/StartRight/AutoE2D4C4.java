package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;

@AutoDescription(description = "21 Point Auto - One L2, Two L4")
public class AutoE2D4C4 extends AutoBase {
  // Start Left Equivalent: AutoJ2K4L4

  private static final PathPlannerPath startingPath = Paths.SR_E2;

  public AutoE2D4C4() {
    super(startingPath.getStartingHolonomicPose());
    System.out.println("========START AUTO");
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());
    addCommands(followPathCommand(startingPath)); // accurate
    addCommands(followPathCommand(Paths.E2_RL));
    addCommands(followPathCommand(Paths.RL_D4));
    addCommands(followPathCommand(Paths.D4_RL));
    // addCommands(followPathCommand(Paths.RL_C4));
  }
}
