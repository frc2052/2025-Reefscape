// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class H4AlgaeGHEFIJ extends AutoBase {

    private static final Path startPath = PathsBase.B_SC_GH;
    private static final Path reposition = PathsBase.B_GH_REPOSITION_IN;
    private static final Path scoreGH = PathsBase.B_GH_NET;

    private static final Path descoreEF = PathsBase.B_NET_EF;
    private static final Path scoreEF = PathsBase.B_EF_NET;

    private static final Path pickupIJ = PathsBase.B_NET_IJ;
    private static final Path moveOffLine = PathsBase.BLUE_NET_FINAL;
    private static final Path scoreIJ = PathsBase.B_IJ_NET;

    public H4AlgaeGHEFIJ() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());

        // score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.GH))
                .andThen(new ParallelCommandGroup(
                        ArmCommandFactory.coralIn().withTimeout(1),
                        ClimberCommandFactory.climberDown().withTimeout(0.5),
                        Commands.sequence(
                                toPosition(TargetAction.HOME),
                                Commands.waitUntil(
                                        () -> !ElevatorSubsystem.getInstance().isHoming()),
                                new WaitCommand(0.3),
                                toPosition(TargetAction.L4)),
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.GH)
                                .withTimeout(2.25)))
                .andThen(score(TargetAction.L4)));

        // pickup GH
        addCommands(toPosition(TargetAction.LOWER_ALGAE).andThen(new WaitCommand(0.2)));
        addCommands(ArmCommandFactory.algaeIn()
                .withDeadline(followPathCommand(reposition.getChoreoPath()).andThen(new WaitCommand(0.2))));

        // score GH
        addCommands((followPathCommand(scoreGH.getChoreoPath()).deadlineFor(ArmCommandFactory.algaeIn()))
                .alongWith(toPosition(TargetAction.ALGAE_NET).beforeStarting(new WaitCommand(0.5)))
                .andThen(scoreNet()));

        // pickup EF
        addCommands(toPosition(TargetAction.UPPER_ALGAE)
                .andThen(((followPathCommand(descoreEF.getChoreoPath()).andThen(new WaitCommand(0.5)))
                                .beforeStarting(new WaitCommand(0.2)))
                        .deadlineFor(ArmCommandFactory.algaeIn().beforeStarting(new WaitCommand(0.2)))));

        // score EF
        addCommands((followPathCommand(scoreEF.getChoreoPath()).deadlineFor(ArmCommandFactory.algaeIn()))
                .andThen(toPosition(TargetAction.ALGAE_NET))
                .andThen(scoreNet()));

        // move off the line
        addCommands((followPathCommand(moveOffLine.getChoreoPath()).beforeStarting(new WaitCommand(0.2)))
                .alongWith(toPosition(TargetAction.POST_ALGAE_STOW)));
    }
}
