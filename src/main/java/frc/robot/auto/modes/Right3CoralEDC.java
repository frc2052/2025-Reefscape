// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

public class Right3CoralEDC extends AutoBase {

    private boolean dScored;
    private static final Path startPath = PathsBase.B_SR_E; //
    private static final Path firstPickup = PathsBase.EXTENDED_E_RL; //
    private static final Path retryLoad = PathsBase.BLUE_RL_RETRY_STRAIGHT;

    private static final Path alignRepos = PathsBase.RIGHT_ALIGN_REPOS;

    public Right3CoralEDC() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    private void setDScored(boolean b) {
        System.out.println("D SET TO: " + b);
        dScored = b;
    }

    @Override
    public void init() {
        // setup
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());
        addCommands(new InstantCommand(() -> setDScored(false)));

        // score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.EF)));
        addCommands(new ParallelCommandGroup(
                (followPathCommand(startPath.getChoreoPath()))
                        .deadlineFor(ArmCommandFactory.coralIn().withTimeout(1)),
                ClimberCommandFactory.climberDown().withTimeout(0.5),
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HOME))));
        addCommands(
                (new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4)))
                        .alongWith((AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.LEFT_BRANCH, FieldElementFace.EF)
                                .withTimeout(3))));
        addCommands(new InstantCommand(() -> System.out.println("start scoring")));
        addCommands(score(TargetAction.L4));

        addCommands(pickup(firstPickup).until(haveCoral()));
        addCommands(new ConditionalCommand(
                followPathCommand(alignRepos.getChoreoPath()), new InstantCommand(), haveCoral()));

        // HAVE FIRST PICKUP CORAL?
        addCommands(new ConditionalCommand(
                // yes? align and score D L4
                Commands.sequence(
                        new PrintCommand("HAVE FIRST PICKUP CORAL: GOING TO SCORE D L4"),
                        new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.CD)),
                        new ParallelCommandGroup(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.CD)
                                                .deadlineFor(IntakeCommandFactory.outtake()),
                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.L4))
                                                .beforeStarting(new WaitCommand(0.5)))
                                .andThen(new InstantCommand(() -> System.out.println("start scoring"))),
                        score(TargetAction.L4),
                        new InstantCommand(() -> setDScored(true))),
                // no? print, move on to reload
                new InstantCommand(() -> System.out.println("FAILED FIRST PICKUP - GOING TO RETRY")),
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3
                        || RobotState.getInstance().getHasCoral())));

        // retry OR reload
        addCommands(pickup(retryLoad).until(haveCoral()));
        addCommands(new ConditionalCommand(
                followPathCommand(alignRepos.getChoreoPath()), new InstantCommand(), haveCoral()));

        // HAVE SECOND PICKUP CORAL?
        addCommands(new ConditionalCommand(
                // yes? score C L4
                Commands.sequence(
                        new PrintCommand("SUCCESSFUL SECOND PICKUP: GOING TO SCORE C L4"),
                        new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L3)),
                        new ParallelCommandGroup(
                                AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.CD)
                                        .deadlineFor(IntakeCommandFactory.outtake()),
                                new SequentialCommandGroup(
                                                new WaitCommand(0.4),
                                                new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.L4)))
                                        .andThen(new InstantCommand(() -> System.out.println("start scoring")))),
                        score(TargetAction.L4)),
                // no? retry load
                new SequentialCommandGroup(
                        new PrintCommand("FAILED SECOND PICKUP: GOING TO RETRY C L4"),
                        pickup(retryLoad).until(haveCoral()),
                        new ConditionalCommand(
                                followPathCommand(alignRepos.getChoreoPath()), new InstantCommand(), haveCoral()),
                        // WE TRIED PICKUP FROM HP AGAIN - NOW DO WE HAVE CORAL?
                        new ConditionalCommand(
                                // yes? score C L4
                                new SequentialCommandGroup(
                                        new PrintCommand(
                                                "SUCCESSFUL RETRY 2ND PICKUP, HAVE CORAL, GOING TO TRY SCORING C L4"),
                                        new ParallelCommandGroup(
                                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                        () -> AlignOffset.LEFT_BRANCH,
                                                                        FieldElementFace.CD)
                                                                .deadlineFor(IntakeCommandFactory.outtake()),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(0.4),
                                                                new InstantCommand(
                                                                        () -> SuperstructureSubsystem.getInstance()
                                                                                .setCurrentAction(TargetAction.L4))))
                                                .andThen(new InstantCommand(() -> System.out.println("start scoring"))),
                                        score(TargetAction.L4)),
                                // no? run reload
                                new PrintCommand("FAILED RETRY 2ND PICKUP, PICKUP AGAIN")
                                        .andThen(
                                                pickup(retryLoad).until(haveCoral()),
                                                new ConditionalCommand(
                                                        followPathCommand(alignRepos.getChoreoPath()),
                                                        new InstantCommand(),
                                                        haveCoral())),
                                // do we have coral? (retry)
                                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3))),
                // conditional for first conditional
                () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3
                        || RobotState.getInstance().getHasCoral())));

        // SCORED D ALREADY?
        // yes?
        // no? --> score D (already ran reload)
        addCommands(new ConditionalCommand(
                // yes? stop
                new InstantCommand(),
                // no? try again - but do you ALREADY HAVE CORAL?
                new ConditionalCommand(
                        // yes, already have coral? --> align and score
                        new SequentialCommandGroup(
                                new PrintCommand("DIDN'T MAKE D AT FIRST, ALREADY HAVE CORAL, TRYING AGAIN!"),
                                new ParallelCommandGroup(
                                                AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.CD)
                                                        .deadlineFor(IntakeCommandFactory.outtake()),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(0.4),
                                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                                .setCurrentAction(TargetAction.L4))))
                                        .andThen(new InstantCommand(() -> System.out.println("start scoring"))),
                                score(TargetAction.L4)),
                        // no, don't have coral? --> reload then score
                        new SequentialCommandGroup(
                                // RELOAD
                                new PrintCommand("DIDN'T MAKE D AT FIRST, NO CORAL, RELOADING!"),
                                pickup(retryLoad).until(haveCoral()),
                                new ConditionalCommand(
                                        followPathCommand(alignRepos.getChoreoPath()),
                                        new InstantCommand(),
                                        haveCoral()),
                                // SCORE
                                new SequentialCommandGroup(
                                        new PrintCommand("DID RELOAD TO RETRY D, GOING TO SCORE!"),
                                        new ParallelCommandGroup(
                                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                                        () -> AlignOffset.RIGHT_BRANCH,
                                                                        FieldElementFace.CD)
                                                                .deadlineFor(IntakeCommandFactory.outtake()),
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(0.4),
                                                                new InstantCommand(
                                                                        () -> SuperstructureSubsystem.getInstance()
                                                                                .setCurrentAction(TargetAction.L4))))
                                                .andThen(new InstantCommand(() -> System.out.println("start scoring"))),
                                        score(TargetAction.L4))),
                        () -> RobotState.getInstance().getHasCoral()),
                () -> dScored));

        // scored all three, final pickup
        addCommands(pickup(retryLoad).until(haveCoral()));
        addCommands(new ConditionalCommand(
                followPathCommand(alignRepos.getChoreoPath()), new InstantCommand(), haveCoral()));
    }
}
