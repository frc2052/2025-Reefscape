package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.drive.AlignWithTagCommand.AlignLocation;
import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocation;

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

    // just paths
    addCommands(followPathCommand(startingPath));
    addCommands(followPathCommand(Paths.E2_RL));
    addCommands(followPathCommand(Paths.RL_D4));
    addCommands(followPathCommand(Paths.D4_RL));
    addCommands(followPathCommand(Paths.RL_C4));

    // test align
    // addCommands(followPathCommand(startingPath));
    // addCommands(followPathCommand(Paths.E2_RL));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.RL_D4, SnapLocation.ReefCD));
    // addCommands(followPathCommand(Paths.D4_RL));
    // addCommands(reefSideVisionOrPathAlign(AlignLocation.LEFT, Paths.RL_C4, SnapLocation.ReefCD));
  }
}
