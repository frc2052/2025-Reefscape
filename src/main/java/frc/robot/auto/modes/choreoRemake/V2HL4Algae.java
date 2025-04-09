// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class V2HL4Algae extends AutoBase {

    private static final Path startPath = PathsBase.B_SC_G;
    private static final Path reposition = PathsBase.B_GH_REPOSITION_OUT;
    private static final Path score1 = PathsBase.B_GH_NET;
    private static final Path descore2 = PathsBase.B_NET_IJ;
    private static final Path score2 = PathsBase.B_IJ_NET;
    private static final Path descore3 = PathsBase.B_NET_KL;
    private static final Path score3 = PathsBase.B_KL_NET;

    public V2HL4Algae() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());

        // score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.GH))
                .andThen(new ParallelCommandGroup(
                        ArmCommandFactory.intake().withTimeout(1),
                        ClimberCommandFactory.climberDown().withTimeout(0.5),
                        new SequentialCommandGroup(
                                new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L3)),
                                new WaitCommand(0.2),
                                new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4))),
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.GH)
                                .withTimeout(3.0)))
                .andThen(score(TargetAction.L4)));

        // pickup GH
        addCommands(ArmCommandFactory.algaeIn()
                .withDeadline(followPathCommand(reposition.getPathPlannerPath()).andThen(new WaitCommand(0.5))));

        // score GH
        addCommands(followPathCommand(score1.getPathPlannerPath())
                .alongWith(new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS))
                        .beforeStarting(new WaitCommand(1.0)))
                .andThen(scoreNet()));

        // pickup IJ
        addCommands(new InstantCommand(
                        () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.UA))
                .andThen((followPathCommand(descore2.getPathPlannerPath()).andThen(new WaitCommand(0.5)))
                        .deadlineFor(ArmCommandFactory.intake().beforeStarting(new WaitCommand(1.0)))));

        // score IJ
        addCommands(followPathCommand(score2.getPathPlannerPath())
                .alongWith(new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS))
                        .beforeStarting(new WaitCommand(1.0)))
                .andThen(scoreNet()));

        // pickup KL
        addCommands(new InstantCommand(
                        () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.UA))
                .andThen((followPathCommand(descore3.getPathPlannerPath()).andThen(new WaitCommand(0.5)))
                        .deadlineFor(ArmCommandFactory.intake().beforeStarting(new WaitCommand(1.0)))));

        // score KL
        addCommands(followPathCommand(score3.getPathPlannerPath())
                .alongWith(new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS))
                        .beforeStarting(new WaitCommand(1.0)))
                .andThen(scoreNet()));
    }
}
