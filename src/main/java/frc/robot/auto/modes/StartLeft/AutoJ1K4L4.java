package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

@AutoDescription(description = "LEFT One L2, Two L4")
public class AutoJ1K4L4 extends AutoBase {
    // Start Right Equivalent: AutoE2D4C4

    private static final PathPlannerPath startingPath = Paths.SL_IJ;

    public AutoJ1K4L4() {
        super(startingPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(delaySelectedTime());
        addCommands(getBumpCommand());

        // alignment L1
        // addCommands(
        //     safeReefAlignment(startingPath, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.IJ)
        //         .alongWith(
        //             prepareForScoreWhenReady(TargetAction.L1H)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L1H)));

        // path L1 (off side)
        // addCommands(followPathCommand(Paths.SL_IJ_L1)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L1H)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L1H)));

        // //
        // addCommands(safeStationAlignment(Paths.J2_LL));
        // addCommands(HPIntake());
        // addCommands(safeReefAlignment(Paths.LL_K4, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L4)));

        // //
        // addCommands(safeStationAlignment(Paths.K4_LL));
        // addCommands(HPIntake());
        // addCommands(HandCommandFactory.motorIn().withTimeout(1.0));
        // addCommands( // don't go  back to reef or start raising elevator
        //     safeReefAlignment(Paths.LL_K4, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.KL)
        //         .alongWith(
        //             prepareForScoreWhenReady(TargetAction.L4)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L4)));

        // 4 coral auto addition - side A
        // addCommands(safeStationAlignment(Paths.L4_LL));
        // addCommands(HPIntake());
        // addCommands(
        //     safeReefAlignment(Paths.LL_AB, AlignOffset.LEFT_REEF_LOC, TargetFieldLocation.AB)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4))
        //         .andThen(HandCommandFactory.motorIn().withTimeout(0.05)));
        // addCommands(score(TargetAction.L4));
    }
}
