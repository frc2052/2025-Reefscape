package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

@AutoDescription(description = "21 Point Auto - One L2, Two L4")
public class AutoE4D4C4 extends AutoBase {
  // Start Left Equivalent: AutoJ2K4L4

  private static final PathPlannerPath startingPath = Paths.SR_E2;

  public AutoE4D4C4() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() { // test!
    addCommands(delaySelectedTime());
    addCommands(getBumpCommand());

    addCommands(
        safeReefAlignment(startingPath, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.EF)
            .alongWith(
                prepareForScoreWhenReady(TargetAction.L4)
                    .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
            .andThen(score(TargetAction.L4)));

    //
    addCommands(safeStationAlignment(Paths.E2_RL));
    addCommands(HPIntake());
    addCommands(
        safeReefAlignment(Paths.RL_D4, AlignOffset.RIGHT_REEF_LOC, TargetFieldLocation.CD)
            .alongWith(
                prepareForScoreWhenReady(TargetAction.L4)
                    .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
            .andThen(score(TargetAction.L4)));

    //
    addCommands(safeStationAlignment(Paths.D4_RL));
    addCommands(HPIntake());
    addCommands(
        safeReefAlignment(Paths.RL_C4, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.CD)
            .alongWith(
                prepareForScoreWhenReady(TargetAction.L4)
                    .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
            .andThen(score(TargetAction.L4)));

    // 4 coral auto addition - side B
    // addCommands(safeStationAlignment(Paths.C4_RL));
    // addCommands(HPIntake());
    // addCommands(
    //     safeReefAlignment(Paths.LL_AB, AlignOffset.RIGHT_REEF_LOC, TargetFieldLocation.AB)
    //         .alongWith(prepareForScoreWhenReady(TargetAction.L4))
    //         .andThen(HandCommandFactory.motorIn().withTimeout(0.05)));
    // addCommands(score(TargetAction.L4));
  }
}
