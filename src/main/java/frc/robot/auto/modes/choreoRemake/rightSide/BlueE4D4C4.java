// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake.rightSide;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoBase.Path;
import frc.robot.auto.common.AutoBase.PathsBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class BlueE4D4C4 extends AutoBase {

    private static final Path startPath = PathsBase.B_SR_E;
    private static final Path load1 = PathsBase.B_E_RL;
    private static final Path score2 = PathsBase.B_RL_D;;
    private static final Path load2 = PathsBase.B_D_RL;
    private static final Path score3 = PathsBase.B_RL_C;

    public BlueE4D4C4() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(delaySelectedTime());
        addCommands(getBumpCommand());

        // score initial choral (E)
        addCommands(safeReefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4))
                .andThen(new PrintCommand("alignment done")
                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.05))
                        .andThen(score(TargetAction.L4))));

        // pickup and score 2nd coral (K)
        addCommands(safeStationAlignment(load1));
        addCommands(new WaitCommand(0.75));
        addCommands(safeReefAlignment(score2, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.05)))
                .andThen(score(TargetAction.L4)));

        // pickup and score 3rd coral (L)
        addCommands(safeStationAlignment(load2));
        addCommands(new WaitCommand(0.75));
        addCommands(safeReefAlignment(score3, AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                .alongWith(prepareForScoreWhenReady(TargetAction.L4)
                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.05)))
                .andThen(score(TargetAction.L4)));
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.STOW)));
    }
}
