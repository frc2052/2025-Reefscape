// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.safety;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
@AutoDescription(description = "CENTER Side")
public class AutoG4AlgaePrep extends AutoBase {

    public static PathPlannerPath startPath = Paths.SC_GH;

    public AutoG4AlgaePrep() {
        super(startPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());

        // addCommands(safeReefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.GH)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4)
        //                 .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
        //         .andThen(score(TargetAction.L4)));
        // addCommands(elevatorToPos(TargetAction.L1H));
        // addCommands(followPathCommand(Paths.G_AlGAE_PREP));
    }
}
