package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;

@AutoDescription(description = "RIGHT One L2, Two L4")
public class AutoF4D4C4 extends AutoBase {
    // Start Left Equivalent: AutI4K4L4

    private static final PathPlannerPath startingPath = Paths.SR_F;

    public AutoF4D4C4() {
        super(startingPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(delaySelectedTime());
        addCommands(getBumpCommand());

        // addCommands(safeReefAlignment(startingPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.EF)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L4)));

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
        // addCommands(safeReefAlignment(Paths.RL_C4, AlignOffset.LEFT_BRANCH, FieldElementFace.CD)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4)
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
