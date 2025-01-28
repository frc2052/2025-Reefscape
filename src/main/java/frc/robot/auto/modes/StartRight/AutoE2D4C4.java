package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.drive.AlignWithReefCommand.AlignLocation;
import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocation;
import frc.robot.commands.superstructure.SuperstructureCommandFactory.ToLevel;

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

    // just paths
    // addCommands(followPathCommand(startingPath));
    // addCommands(scoreLevel(ToLevel.L2));
    // addCommands(followPathCommand(Paths.E2_RL));
    // addCommands(followPathCommand(Paths.RL_D4));
    // addCommands(scoreLevel(ToLevel.L4));
    // addCommands(followPathCommand(Paths.D4_RL));
    // addCommands(followPathCommand(Paths.RL_C4));
    // addCommands(scoreLevel(ToLevel.L4));

    // vision align
    addCommands(followPathCommand(startingPath));
    addCommands(scoreLevel(ToLevel.L2));
    addCommands(followPathCommand(Paths.E2_RL));
    addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.RL_D4, SnapLocation.ReefCD));
    addCommands(scoreLevel(ToLevel.L4));
    addCommands(followPathCommand(Paths.D4_RL));
    addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.RL_C4, SnapLocation.ReefCD));
    addCommands(scoreLevel(ToLevel.L4));
  }
}
