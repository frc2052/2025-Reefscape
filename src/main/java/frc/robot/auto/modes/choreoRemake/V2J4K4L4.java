// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

public class V2J4K4L4 extends AutoBase {

    private enum CommandSelector {
        ALIGN_AND_SCORE,
        RETRY
    }

    private CommandSelector select() {
        if (RobotState.getInstance().getHasCoral()) {
            return CommandSelector.ALIGN_AND_SCORE;
        } else {
            return CommandSelector.RETRY;
        }
    }

    private boolean kScored;
    private static final Path startPath = PathsBase.B_SL_J;
    private static final Path load1 = PathsBase.EXTENDED_J_LL;
    private static final Path retryLoad = PathsBase.BLUE_LL_RETRY;

    public V2J4K4L4() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    private void setKScored(boolean b) {
        kScored = b;
    }

    @Override
    public void init() {
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());
        addCommands(new InstantCommand(() -> setKScored(false))); // rest kScored to false

        // faster score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.IJ))
                .andThen(followPathCommand(startPath.getChoreoPath())
                        .andThen((new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.L4))
                                        .beforeStarting(new WaitCommand(0.3)))
                                .alongWith(IntakeCommandFactory.intake().withTimeout(0.5))
                                .alongWith(AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ))))
                .andThen(score(TargetAction.L4)));

        // go down safely
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                        .andThen(new WaitCommand(0.15)));

        addCommands((followPathCommand(load1.getPathPlannerPath())
                        .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                .until(() -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // have coral?
        addCommands(new ConditionalCommand(
                // yes? align and score K L4
                Commands.sequence(
                        new PrintCommand("HAVE CORAL"),
                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                                        .andThen(new WaitCommand(0.2))) // added a wait after alignment
                                .alongWith(new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4))),
                        score(TargetAction.L4),
                        new InstantCommand(() -> setKScored(true)),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.4))),
                // no? print, move on to reload
                new InstantCommand(() -> System.out.println("NO CORAL")),
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction()
                        == TargetAction.STOW))); // goes to stow if we have coral

        // retry / reload
        addCommands((followPathCommand(retryLoad.getPathPlannerPath())
                        .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                .until(() -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // have coral?
        addCommands(new ConditionalCommand(
                // yes? score L L4
                Commands.sequence(
                        new PrintCommand("HAVE CORAL"),
                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                                        .andThen(new WaitCommand(0.4)))
                                .alongWith(new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4))),
                        score(TargetAction.L4),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.5))),
                // no? retry load
                new SequentialCommandGroup(
                        (followPathCommand(retryLoad.getPathPlannerPath())
                                        .deadlineFor(
                                                IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                                .until(() ->
                                        (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                                                || RobotState.getInstance().getHasCoral())),
                        // now do we have coral?
                        new ConditionalCommand(
                                // yes? score L L4
                                new SequentialCommandGroup(
                                        new PrintCommand("HAVE CORAL"),
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                                                .alongWith(new InstantCommand(
                                                        () -> SuperstructureSubsystem.getInstance()
                                                                .setCurrentAction(TargetAction.L4))),
                                        score(TargetAction.L4),
                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.INTAKE))
                                                .andThen(new WaitCommand(0.4))),
                                // no? run reload
                                (followPathCommand(retryLoad.getPathPlannerPath())
                                                .deadlineFor(IntakeCommandFactory.intake()
                                                        .alongWith(ArmCommandFactory.intake())))
                                        .until(() -> (SuperstructureSubsystem.getInstance()
                                                                .getCurrentAction()
                                                        == TargetAction.STOW
                                                || RobotState.getInstance().getHasCoral())),
                                // do we have coral
                                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW))),
                // conditional for first conditional
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW)));

        // scored k?
        // yes? stop. next auto --> load lolipop and score A (4 coral)
        // no? --> score K (alread ran reload)
    }
}
