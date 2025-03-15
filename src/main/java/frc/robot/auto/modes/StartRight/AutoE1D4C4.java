package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;

@AutoDescription(description = "RIGHT One L2, Two L4")
public class AutoE1D4C4 extends AutoBase {
    // Start Left Equivalent: AutoJ2K4L4

    private static final PathPlannerPath startingPath = Paths.SR_EF;

    public AutoE1D4C4() {
        super(startingPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(delaySelectedTime());
        addCommands(getBumpCommand());

        // alignment L1
        // addCommands(
        //     safeReefAlignment(startingPath, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.EF)
        //         .alongWith(
        //             prepareForScoreWhenReady(TargetAction.L1H)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L1H)));

        // paths L1
        // addCommands(followPathCommand(Paths.SR_EF_L1)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L1H)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L1H)));

        // //
        // addCommands(safeStationAlignment(Paths.E2_RL));
        // addCommands(HPIntake());
        // addCommands(safeReefAlignment(Paths.RL_D4, AlignOffset.RIGHT_BRANCH, FieldElementFace.CD)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L4)));

        // //
        // addCommands(safeStationAlignment(Paths.D4_RL));
        // addCommands(HPIntake());
        // addCommands(HandCommandFactory.motorIn().withTimeout(1.0));
        // addCommands( // don't keep going
        //     safeReefAlignment(Paths.RL_C4, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.CD)
        //         .alongWith(
        //             prepareForScoreWhenReady(TargetAction.L4)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L4)));

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
