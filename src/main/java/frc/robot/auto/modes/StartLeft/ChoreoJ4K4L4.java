// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.StartLeft;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class ChoreoJ4K4L4 extends AutoBase {
    // we pass in Path and followPath turns them into pp paths

    private static final Path startPath = PathsBase.B_SL_J;
    ;
    private static final Path load1 = PathsBase.B_J_LL;
    private static final Path score2 = PathsBase.B_LL_K;
    ;
    private static final Path load2 = PathsBase.B_K_LL;
    private static final Path score3 = PathsBase.B_LL_K;
    ;

    public ChoreoJ4K4L4() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(delaySelectedTime());
        addCommands(getBumpCommand());

        // path works, alignment off
        addCommands(safeReefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4))
                .andThen(new PrintCommand("alignment done")
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.05))
                        // .andThen(new DefaultDriveCommand(() -> 0.7, () -> 0, () -> 0, () -> false))
                        .andThen(score(TargetAction.L4))));

        //
        addCommands(safeStationAlignment(load1));
        addCommands(new WaitCommand(0.75));
        addCommands(safeReefAlignment(score2, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
                .andThen(score(TargetAction.L4)));

        //
        addCommands(safeStationAlignment(load2));
        addCommands(new WaitCommand(0.75));
        addCommands(safeReefAlignment(score3, AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
                .andThen(score(TargetAction.L4)));
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HP)));
    }
}
