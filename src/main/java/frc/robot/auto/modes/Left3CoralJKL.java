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

public class Left3CoralJKL extends AutoBase {

    private boolean kScored;
    private static final Path startPath = PathsBase.B_SL_J;
    private static final Path firstPickup = PathsBase.EXTENDED_J_LL;
    private static final Path retryLoad = PathsBase.BLUE_LL_RETRY_STRAIGHT;

    private static final Path alignRepos = PathsBase.LEFT_ALIGN_REPOS;

    private static final Path KLreposition = PathsBase.B_KL_REPOSITION;

    public Left3CoralJKL() {
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
        addCommands(new InstantCommand(() -> setKScored(false)));

        // score preload
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.IJ)));
        addCommands(new ParallelCommandGroup(
                (followPathCommand(startPath.getChoreoPath()))
                        .deadlineFor(ArmCommandFactory.coralIn().withTimeout(1)),
                ClimberCommandFactory.climberDown().withTimeout(0.5),
                superstructure.set(TargetAction.HOME, true)));
        addCommands(superstructure
                .set(TargetAction.L4, true)
                .alongWith((AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)
                        .withTimeout(3))));

        addCommands(new InstantCommand(() -> System.out.println("start scoring")));
        addCommands(score(TargetAction.L4));

        addCommands(Commands.sequence(Commands.repeatingSequence(pickup(retryLoad)).until(haveCoral()), followPathCommand(alignRepos.getChoreoPath())).unless(haveCoral()));
        addCommands(alignAndScore(AlignOffset.LEFT_BRANCH, FieldElementFace.KL, TargetAction.L4).unless(() -> kScored));

        addCommands(Commands.sequence(Commands.repeatingSequence(pickup(retryLoad)).until(haveCoral()), followPathCommand(alignRepos.getChoreoPath())).unless(haveCoral()));
        addCommands(alignAndScore(AlignOffset.RIGHT_BRANCH, FieldElementFace.KL, TargetAction.L4));
 

        // scored all three, final pickup
        addCommands(pickup(retryLoad).until(haveCoral()));
        addCommands(new ConditionalCommand(
                followPathCommand(alignRepos.getChoreoPath()), new InstantCommand(), haveCoral()));
    }
}
