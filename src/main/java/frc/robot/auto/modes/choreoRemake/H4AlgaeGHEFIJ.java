// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake;

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
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class H4AlgaeGHEFIJ extends AutoBase {

    private static final Path startPath = PathsBase.B_SC_G;
    private static final Path reposition = PathsBase.B_GH_REPOSITION_IN;
    private static final Path scoreGH = PathsBase.B_GH_NET;

    private static final Path descoreEF = PathsBase.B_NET_EF;
    private static final Path scoreEF = PathsBase.B_EF_NET;

    private static final Path netForward = PathsBase.BLUE_NET_FORWARD;
    private static final Path pickupIJ = PathsBase.B_NET_IJ;
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
                        ArmCommandFactory.intake().withTimeout(1),
                        ClimberCommandFactory.climberDown().withTimeout(0.5),
                        Commands.sequence(
                                new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HM)),
                                Commands.waitUntil(
                                        () -> !ElevatorSubsystem.getInstance().isHoming()),
                                new WaitCommand(0.3),
                                new InstantCommand(() ->
                                        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4))),
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.GH)
                                .withTimeout(2.25)))
                .andThen(score(TargetAction.L4)));

        // pickup GH
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.LA))
                        .andThen(new WaitCommand(0.2)));
        addCommands(ArmCommandFactory.algaeIn()
                .withDeadline(followPathCommand(reposition.getChoreoPath()).andThen(new WaitCommand(0.2))));

        // score GH
        addCommands((followPathCommand(scoreGH.getChoreoPath()).deadlineFor(ArmCommandFactory.algaeIn()))
                .alongWith(new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS))
                        .beforeStarting(new WaitCommand(0.5)))
                .andThen(scoreNet()));

        // pickup EF
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.UA))
                        .andThen(((followPathCommand(descoreEF.getChoreoPath()).andThen(new WaitCommand(0.5)))
                                        .beforeStarting(new WaitCommand(0.2)))
                                .deadlineFor(ArmCommandFactory.algaeIn().beforeStarting(new WaitCommand(0.2)))));

        // score EF
        addCommands((followPathCommand(scoreEF.getChoreoPath()).deadlineFor(ArmCommandFactory.algaeIn()))
                .andThen(new InstantCommand(
                        () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS)))
                .andThen(scoreNet()));

        // pickup IJ
        addCommands(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.UA))
                        .andThen(((followPathCommand(pickupIJ.getChoreoPath()).andThen(new WaitCommand(0.5)))
                                        .beforeStarting(new WaitCommand(0.2)))
                                .deadlineFor(ArmCommandFactory.algaeIn().beforeStarting(new WaitCommand(0.2)))));

        // score IJ
        addCommands((followPathCommand(scoreIJ.getChoreoPath()).deadlineFor(ArmCommandFactory.algaeIn()))
                .andThen(new InstantCommand(
                        () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS)))
                .andThen(scoreNet()));

        // alternative safety
        // addCommands(
        //         new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.STOW))
        //                 .alongWith(followPathCommand(netForward.getChoreoPath()))); // before scoring
    }
}
