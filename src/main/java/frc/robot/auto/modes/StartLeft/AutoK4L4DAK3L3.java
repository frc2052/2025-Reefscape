package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;

@AutoDescription(description = "LEFT Two L4, Remove Algae, Two L3")
public class AutoK4L4DAK3L3 extends AutoBase {
    // Start Right Equivalent: AutoSLK4L4DAK3L3

    private static final PathPlannerPath startingPath = Paths.SL_K4;

    public AutoK4L4DAK3L3() {
        super(startingPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        // addCommands(delaySelectedTime());
        // addCommands(getBumpCommand());

        addCommands(startHP());
        // addCommands(safeReefAlignment(startingPath, AlignOffset.LEFT_BRANCH, FieldElementFace.KL));
        // addCommands(toPosAndScore(TargetAction.L4));
        // addCommands(safeStationAlignment(Paths.K4_LL));
        // addCommands(safeReefAlignment(Paths.LL_L4, AlignOffset.RIGHT_BRANCH, FieldElementFace.KL));
        // addCommands(
        //         descoreScoreNetAlgae(Paths.KL_SCORE_TO_DESCORE, TargetAction.UA, Paths.KL_NET)); // KL is upper algae
        // addCommands(safeStationAlignment(Paths.NET_SCORE_LEFT_STATION));
        // addCommands(safeReefAlignment(Paths.LL_K3, AlignOffset.LEFT_BRANCH, FieldElementFace.KL));
        // addCommands(toPosAndScore(TargetAction.L3));
        // addCommands(safeStationAlignment(Paths.K3_LL));
        // addCommands(safeReefAlignment(Paths.LL_L3, AlignOffset.RIGHT_BRANCH, FieldElementFace.KL));
    }
}
