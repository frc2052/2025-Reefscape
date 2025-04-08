// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.startCenter;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;

/** Add your docs here. */
@AutoDescription(description = "CENTER Side")
public class AutoH4RightAlgaeRemoval extends AutoBase {
    public static final PathPlannerPath startPath = Paths.SC_H4;

    public AutoH4RightAlgaeRemoval() {
        super(startPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());

        // addCommands(safeReefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.GH));
        addCommands(toPosAndScore(TargetAction.L4));
        // addCommands(scoreNet(Paths.GH_SCORE_TO_DESCORE, TargetAction.LA, Paths.GH_NET));
        // addCommands(scoreNet(Paths.NET_EF, TargetAction.UA, Paths.EF_NET));
        // addCommands(scoreNet(Paths.NET_CD, TargetAction.LA, Paths.CD_NET));
    }
}
