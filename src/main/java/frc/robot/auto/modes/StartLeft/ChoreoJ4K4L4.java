// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.common.AutoBase;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class ChoreoJ4K4L4 extends AutoBase{

    private static final PathPlannerPath startPath = PathsBase.SL_J2.getChoreoPath();
    private static final PathPlannerPath load1 = PathsBase.J2_LL.getChoreoPath();
    private static final PathPlannerPath score2 = PathsBase.LL_K4.getChoreoPath();
    private static final PathPlannerPath load2 = PathsBase.K4_LL.getChoreoPath();
    private static final PathPlannerPath score3 = PathsBase.LL_K4.getChoreoPath();

    public ChoreoJ4K4L4(){
        super(startPath.getStartingHolonomicPose());
    }

    @Override
    public void init() {
         addCommands(delaySelectedTime());
        addCommands(getBumpCommand());

        //
        addCommands(safeReefAlignment(startPath, AlignOffset.LEFT_BRANCH, FieldElementFace.IJ)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
                // .andThen(new DefaultDriveCommand(() -> 0.7, () -> 0, () -> 0, () -> false))
                .andThen(score(TargetAction.L4)));

        //
        addCommands(safeStationAlignment(load1));
        addCommands(HPIntake());
        addCommands(safeReefAlignment(score2, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
                .andThen(score(TargetAction.L4)));

        //
        addCommands(safeStationAlignment(load2));
        addCommands(HPIntake());
        addCommands(safeReefAlignment(score3, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
                .andThen(score(TargetAction.L4)));
    }

    
}
