// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

        // faster score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.IJ))
                .andThen(followPathCommand(startPath.getChoreoPath())
                        .andThen((new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.L4))
                                        .beforeStarting(new WaitCommand(0.5)))
                                .alongWith(AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ))))
                .andThen(score(TargetAction.L4)));

        // score preload J: consistent (for now)
        // addCommands(safeReefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)
        //         .alongWith(prepareForScoreWhenReady(TargetAction.L4)) // note: go up faster?
        //         .andThen(new PrintCommand("J RIGHT ALIGNMENT DONE")
        //                 .andThen(ArmCommandFactory.coralIn().withTimeout(0.05))
        //                 .andThen(score(TargetAction.L4))));

        // go down safely
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                        .andThen(new WaitCommand(0.5)));

        addCommands(followPathCommand(load1.getPathPlannerPath())
                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())));

        // 3/26 pickup successful

        // have coral?
        addCommands(Commands.either(
                // yes? --> score K L4
                Commands.sequence(
                        new InstantCommand(() -> System.out.print("**************NO 1st PICKUP CORAL")),
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.KL),
                        score(TargetAction.L4),
                        new InstantCommand(() -> setKScored(true)),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.5))),
                // no? --> next step (reload, score L L4)
                Commands.sequence(
                        new InstantCommand(() -> System.out.print("**************NO 1st PICKUP CORAL")),
                        new InstantCommand(() -> setKScored(false))),
                () -> RobotState.getInstance().getHasCoral()));

        addCommands(followPathCommand(retryLoad.getChoreoPath())
                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())));

        // have coral?
        Commands.either(
                //  yes? --> score L L4
                Commands.sequence(
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL),
                        score(TargetAction.L4),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.5))),
                // no? --> retry load
                Commands.sequence(
                        followPathCommand(retryLoad.getChoreoPath())
                                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())),
                        // now do we have coral?
                        Commands.either(
                                // yes? score L L4
                                Commands.sequence(
                                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.KL),
                                        score(TargetAction.L4),
                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                        .setCurrentAction(TargetAction.INTAKE))
                                                .andThen(new WaitCommand(0.5))),
                                // no? move on to try K or A
                                new InstantCommand(),
                                () -> RobotState.getInstance().getHasCoral())),
                () -> RobotState.getInstance().getHasCoral());

        // did we ever make k?
        Commands.either(
                // yeah? go for A w/ the left lolipop coral (not implemented)
                new InstantCommand(),
                // no? rip, try again
                Commands.sequence(
                        followPathCommand(retryLoad.getChoreoPath())
                                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())),
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.KL),
                        score(TargetAction.L4),
                        new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                                .andThen(new WaitCommand(0.5))),
                () -> kScored);
    }
}
