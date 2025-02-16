package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.ReefSubSide;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetFieldLocation;


@AutoDescription(description = "21 Point Auto - One L2, Two L4")
public class AutoJ2K4L4 extends AutoBase {
  // Start Right Equivalent: AutoE2D4C4

  private static final PathPlannerPath startingPath = Paths.SL_J2;

  public AutoJ2K4L4() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());

    addCommands(followPathCommand(startingPath));
    addCommands(toPosition(TargetAction.L2));
    addCommands(followPathCommand(Paths.J2_LL));
    addCommands(reefSideVisionOrPathAlign(ReefSubSide.LEFT, Paths.LL_K4, TargetFieldLocation.KL));
    addCommands(followPathCommand(Paths.LL_K4));
    addCommands(toPosition(TargetAction.L4));
    addCommands(followPathCommand(Paths.K4_LL));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.RIGHT, Paths.LL_L4,
    // SnapLocation.ReefKL));
    addCommands(followPathCommand(Paths.LL_K4));
    addCommands(toPosition(TargetAction.L4));
  }
}
