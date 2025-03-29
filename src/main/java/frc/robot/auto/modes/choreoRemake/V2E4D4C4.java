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

public class V2E4D4C4 extends AutoBase {

    private boolean dScored;
    private static final Path startPath = PathsBase.B_SR_E; //
    private static final Path load1 = PathsBase.EXTENDED_E_RL; //
    private static final Path retryLoad = PathsBase.BLUE_RL_RETRY_STRAIGHT; //
    private static final Path rightLolipopPickup = PathsBase.BLUE_RL_LOLIPOP;

    public V2E4D4C4() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    private void setKScored(boolean b) {
        System.out.println("K SET TO: " + b);
        dScored = b;
    }

    @Override
    public void init() {
        // setup
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());
        addCommands(new InstantCommand(() -> setKScored(false))); // rest kScored to false
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.STOW)));

        // score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.EF))
                .andThen(followPathCommand(startPath.getChoreoPath()))
                // align w/ extra time + raise elevator after delay
                .andThen((new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4))
                                .beforeStarting(new WaitCommand(0.6)))
                        .alongWith(IntakeCommandFactory.intake().withTimeout(0.5))
                        .alongWith(AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.EF)))
                // check position and score --> slight wait for elevator to stop shaking
                .andThen(score(TargetAction.L4)));

        // go down safely
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                        .andThen(new WaitCommand(0.2)));

        // first pickup from coral station
        addCommands((followPathCommand(load1.getPathPlannerPath()) // working @ straightRetry
                        .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                .until(() -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral()))
                .andThen(new WaitCommand(0.15)));

        // _______________ START TIME CUT HERE

        // HAVE FIRST PICKUP CORAL?
        addCommands(new ConditionalCommand(
                // yes? align and score D L4
                Commands.sequence(
                        new PrintCommand("HAVE FIRST PICKUP CORAL: GOING TO SCORE D L4"),
                        new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.CD)),
                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.CD)
                                        .beforeStarting(new WaitCommand(0.3))
                                        .andThen(new WaitCommand(0.1))) // added a wait after alignment
                                .alongWith(new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.L4))
                                        .beforeStarting(new WaitCommand(0.6))),
                        score(TargetAction.L4),
                        new InstantCommand(() -> setKScored(true)),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.05))),
                // no? print, move on to reload
                new InstantCommand(() -> System.out.println("FAILED FIRST PICKUP - GOING TO RETRY")),
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // retry OR reload
        addCommands(((followPathCommand(retryLoad.getPathPlannerPath())) // give max time to reload
                        .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                .until(() -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // HAVE SECOND PICKUP CORAL?
        addCommands(new ConditionalCommand(
                // yes? score C L4
                Commands.sequence(
                        new PrintCommand("SUCCESSFUL SECOND PICKUP: GOING TO SCORE C L4"),
                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.CD)
                                        .beforeStarting(new WaitCommand(0.3)))
                                .alongWith(new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.L4))
                                        .beforeStarting(new WaitCommand(0.7))),
                        score(TargetAction.L4),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.05))),
                // no? retry load
                new SequentialCommandGroup(
                        new PrintCommand("FAILED SECOND PICKUP: GOING TO RETRY C L4"),
                        ((followPathCommand(retryLoad.getPathPlannerPath()).beforeStarting(new WaitCommand(0.2)))
                                        .deadlineFor(
                                                IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                                .until(() ->
                                        (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                                                || RobotState.getInstance().getHasCoral())),
                        // WE TRIED PICKUP FROM HP AGAIN - NOW DO WE HAVE CORAL?
                        new ConditionalCommand(
                                // yes? score C L4
                                new SequentialCommandGroup(
                                        new PrintCommand(
                                                "SUCCESSFUL RETRY 2ND PICKUP, HAVE CORAL, GOING TO TRY SCORING C L4"),
                                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.CD)
                                                        .beforeStarting(new WaitCommand(0.3))
                                                        .andThen(new WaitCommand(0.15)))
                                                .alongWith(new InstantCommand(
                                                                () -> SuperstructureSubsystem.getInstance()
                                                                        .setCurrentAction(TargetAction.L4))
                                                        .beforeStarting(new WaitCommand(0.7))),
                                        score(TargetAction.L4),
                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.INTAKE))
                                                .andThen(new WaitCommand(0.05))),
                                // no? run reload and score
                                new PrintCommand("FAILED RETRY 2ND PICKUP, PICKUP AGAIN")
                                        .andThen(((followPathCommand(retryLoad.getPathPlannerPath())
                                                                .beforeStarting(new WaitCommand(0.3)))
                                                        .deadlineFor(IntakeCommandFactory.intake()
                                                                .alongWith(ArmCommandFactory.intake())))
                                                .until(() -> (SuperstructureSubsystem.getInstance()
                                                                        .getCurrentAction()
                                                                == TargetAction.STOW
                                                        || RobotState.getInstance()
                                                                .getHasCoral()))),
                                // do we have coral? (retry)
                                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW))),
                // conditional for first conditional
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // SCORED D ALREADY?
        // yes?
        // no? --> score D (already ran reload)
        addCommands(new ConditionalCommand(
                // yes? stop. (4 coral auto --> load lolipop / score B)
                new InstantCommand(),
                // no? try again - but do you ALREADY HAVE CORAL?
                new ConditionalCommand(
                        // yes, already have coral? --> align and score
                        new SequentialCommandGroup(
                                new PrintCommand("DIDN'T MAKE D AT FIRST, ALREADY HAVE CORAL, TRYING AGAIN!"),
                                (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.CD)
                                                .beforeStarting(new WaitCommand(0.3)))
                                        .alongWith(new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.L4))
                                                .beforeStarting(new WaitCommand(0.7))),
                                score(TargetAction.L4),
                                new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.INTAKE))
                                        .andThen(new WaitCommand(0.15))),
                        // no, don't have coral? --> reload then score
                        new SequentialCommandGroup(
                                // RELOAD
                                new PrintCommand("DIDN'T MAKE D AT FIRST, NO CORAL, RELOADING!"),
                                ((followPathCommand(retryLoad.getPathPlannerPath())
                                                        .beforeStarting(new WaitCommand(0.5)))
                                                .deadlineFor(IntakeCommandFactory.intake()
                                                        .alongWith(ArmCommandFactory.intake())))
                                        .until(() -> (SuperstructureSubsystem.getInstance()
                                                                .getCurrentAction()
                                                        == TargetAction.STOW
                                                || RobotState.getInstance().getHasCoral())),
                                // SCORE
                                new SequentialCommandGroup(
                                        new PrintCommand("DID RELOAD TO RETRY D, GOING TO SCORE!"),
                                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.CD)
                                                        .beforeStarting(new WaitCommand(0.3)))
                                                .alongWith(new InstantCommand(
                                                                () -> SuperstructureSubsystem.getInstance()
                                                                        .setCurrentAction(TargetAction.L4))
                                                        .beforeStarting(new WaitCommand(0.7))),
                                        score(TargetAction.L4),
                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.INTAKE))
                                                .andThen(new WaitCommand(0.15)))),
                        () -> RobotState.getInstance().getHasCoral()),
                () -> dScored));

        // 4 coral auto: made E, D, and C, moving on to score B! (left or middle lolipop depending on C pickup)
    }
}
