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

    private boolean kScored;
    private static final Path startPath = PathsBase.B_SL_J;
    private static final Path load1 = PathsBase.EXTENDED_J_LL;
    private static final Path retryLoad = PathsBase.BLUE_LL_RETRY;
    private static final Path straightRetryLoad = PathsBase.BLUE_LL_RETRY_STRAIGHT;

    public V2J4K4L4() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    private void setKScored(boolean b) {
        System.out.println("K SET TO: " + b);
        kScored = b;
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
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.IJ))
                // follow path close to face
                .andThen(followPathCommand(startPath.getChoreoPath()))
                // align w/ extra time + raise elevator after delay
                .andThen((new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4))
                                .beforeStarting(new WaitCommand(0.6)))
                        .alongWith(IntakeCommandFactory.intake().withTimeout(0.5))
                        .alongWith(AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)))
                // check position and score --> added slight wait for elevator to stop shaking
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

        // _______________

        // HAVE FIRST PICKUP CORAL?
        addCommands(new ConditionalCommand(
                // yes? align and score K L4
                Commands.sequence(
                        new PrintCommand("HAVE FIRST PICKUP CORAL: GOING TO SCORE K L4"),
                        new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.KL)),
                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
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

        // if scored K, max had time, no wait
        // if didn't score K, give max more time
        // retry OR reload
        addCommands(((followPathCommand(straightRetryLoad.getPathPlannerPath())
                                .beforeStarting(new WaitCommand(kScored ? 0.0 : 0.2))) // give max time to reload
                        .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                .until(() -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // HAVE SECOND PICKUP CORAL?
        addCommands(new ConditionalCommand(
                // yes? score L L4
                Commands.sequence(
                        new PrintCommand("SUCCESSFUL SECOND PICKUP: GOING TO SCORE L L4"),
                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                                        .beforeStarting(new WaitCommand(0.3))
                                        .andThen(new WaitCommand(0))) // more time for alignment
                                .alongWith(new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.L4))
                                        .beforeStarting(new WaitCommand(0.7))),
                        score(TargetAction.L4),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.05))),
                // no? retry load
                new SequentialCommandGroup(
                        new PrintCommand("FAILED SECOND PICKUP: GOING TO RETRY L L4"),
                        ((followPathCommand(straightRetryLoad.getPathPlannerPath())
                                                .beforeStarting(new WaitCommand(0.2)))
                                        .deadlineFor(
                                                IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
                                .until(() ->
                                        (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                                                || RobotState.getInstance().getHasCoral())),
                        // WE TRIED PICKUP FROM HP AGAIN - NOW DO WE HAVE CORAL?
                        new ConditionalCommand(
                                // yes? score L L4
                                new SequentialCommandGroup(
                                        new PrintCommand(
                                                "SUCCESSFUL RETRY 2ND PICKUP, HAVE CORAL, GOING TO TRY SCORING L L4"),
                                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
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
                                        .andThen(((followPathCommand(straightRetryLoad.getPathPlannerPath())
                                                                .beforeStarting(new WaitCommand(0.3)))
                                                        .deadlineFor(IntakeCommandFactory.intake()
                                                                .alongWith(ArmCommandFactory.intake())))
                                                .until(() -> (SuperstructureSubsystem.getInstance()
                                                                        .getCurrentAction()
                                                                == TargetAction.STOW
                                                        || RobotState.getInstance()
                                                                .getHasCoral()))),
                                // .andThen(new SequentialCommandGroup(
                                //         new PrintCommand("THIRD TRY, SCORE L"),
                                //         AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                //                         () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                                //                 .alongWith(new InstantCommand(
                                //                         () -> SuperstructureSubsystem.getInstance()
                                //                                 .setCurrentAction(TargetAction.L4))),
                                //         score(TargetAction.L4),
                                //         new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                //                         .setCurrentAction(TargetAction.INTAKE))
                                //                 .andThen(new WaitCommand(0.25)))),
                                // do we have coral? (retry)
                                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW))),
                // conditional for first conditional
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                        || RobotState.getInstance().getHasCoral())));

        // SCORED K ALREADY?
        // yes?
        // no? --> score K (alread ran reload)
        addCommands(new ConditionalCommand(
                // yes? stop. next auto --> load lolipop and score A (4 coral)
                new InstantCommand(),
                // no? try again - but do you ALREADY HAVE CORAL?
                new ConditionalCommand(
                        // yes, already have coral? --> align and score
                        new SequentialCommandGroup(
                                new PrintCommand("DIDN'T MAKE K AT FIRST, ALREADY HAVE CORAL, TRYING AGAIN!"),
                                (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                        () -> AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
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
                                new PrintCommand("DIDN'T MAKE K AT FIRST, NO CORAL, RELOADING!"),
                                ((followPathCommand(straightRetryLoad.getPathPlannerPath())
                                                        .beforeStarting(new WaitCommand(0.5)))
                                                .deadlineFor(IntakeCommandFactory.intake()
                                                        .alongWith(ArmCommandFactory.intake())))
                                        .until(() -> (SuperstructureSubsystem.getInstance()
                                                                .getCurrentAction()
                                                        == TargetAction.STOW
                                                || RobotState.getInstance().getHasCoral())),
                                // SCORE
                                new SequentialCommandGroup(
                                        new PrintCommand("RELOAD TO RETRY K, GOING TO SCORE!"),
                                        (AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
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
                () -> kScored));

        // 4 coral auto: made J, K, and L, moving on to score A! (left or middle lolipop depending on L pickup)
    }
}
