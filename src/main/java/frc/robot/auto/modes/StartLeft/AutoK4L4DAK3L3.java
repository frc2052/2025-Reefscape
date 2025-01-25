package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;

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

    // just paths
    addCommands(followPathCommand(startingPath));
    addCommands(followPathCommand(Paths.K4_LL)); //
    addCommands(followPathCommand(Paths.LL_L4)); //
    addCommands(followPathCommand(Paths.L4_LL)); //
    addCommands(followPathCommand(Paths.LL_K3));
    addCommands(followPathCommand(Paths.K3_LL));
    addCommands(followPathCommand(Paths.LL_L3));

    // test align
    // addCommands(followPathCommand(startingPath));
    // addCommands(followPathCommand(Paths.K4_LL));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.RIGHT, Paths.LL_L4,
    // SnapLocation.ReefKL));
    // addCommands(followPathCommand(Paths.L4_LL));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.LL_K3, SnapLocation.ReefKL));
    // addCommands(followPathCommand(Paths.K3_LL));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.RIGHT, Paths.LL_L3,
    // SnapLocation.ReefKL));
  }
}
