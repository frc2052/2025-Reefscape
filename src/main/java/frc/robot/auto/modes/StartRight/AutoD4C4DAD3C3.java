package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.drive.AlignWithReefCommand.AlignLocation;
import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocation;
import frc.robot.commands.superstructure.SuperstructureCommandFactory.ToLevel;

@AutoDescription(description = "29 Point Auto - Two L4, Remove Algae, Two L3")
public class AutoD4C4DAD3C3 extends AutoBase {
  // Start Left Equivalent: AutoK4L4DAK3L3

  private static final PathPlannerPath startingPath = Paths.SR_D4;

  public AutoD4C4DAD3C3() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());

    // just paths
    // addCommands(followPathCommand(startingPath));
    // addCommands(scoreLevel(ToLevel.L4));
    // addCommands(followPathCommand(Paths.D4_RL));
    // addCommands(followPathCommand(Paths.RL_C4));
    // addCommands(scoreLevel(ToLevel.L4));
    // addCommands(followPathCommand(Paths.C4_RL));
    // addCommands(followPathCommand(Paths.RL_D3));
    // addCommands(scoreLevel(ToLevel.L3));
    // addCommands(followPathCommand(Paths.D3_RL));
    // addCommands(followPathCommand(Paths.RL_C3));
    // addCommands(scoreLevel(ToLevel.L3));

    // vision align
    addCommands(followPathCommand(startingPath));
    addCommands(scoreLevel(ToLevel.L4));
    addCommands(followPathCommand(Paths.D4_RL));
    addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.RL_C4, SnapLocation.ReefCD));
    addCommands(scoreLevel(ToLevel.L4));
    addCommands(followPathCommand(Paths.C4_RL));
    addCommands(reefSideVisionOrPathAlign(AlignLocation.RIGHT, Paths.RL_D3, SnapLocation.ReefCD));
    addCommands(scoreLevel(ToLevel.L3));
    addCommands(followPathCommand(Paths.D3_RL));
    addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.RL_C3, SnapLocation.ReefCD));
    addCommands(scoreLevel(ToLevel.L3));
  }
}
