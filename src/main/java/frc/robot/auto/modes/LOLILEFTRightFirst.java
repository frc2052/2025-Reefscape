// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class LOLILEFTRightFirst extends AutoBase {
    // spotless:off

    boolean aScored;
    boolean leftFirst;
    private static final Path startPath = PathsBase.SL_A;
    private static final Path loadCenter = PathsBase.AB_LOLIPOP_C;
    private static final Path loadLeft = PathsBase.AB_LOLIPOP_L;
    private static final Path loadRight = PathsBase.AB_LOLIPOP_R;

    public LOLILEFTRightFirst() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    private void setAScored(boolean b) {
        System.out.println("aScored SET TO: " + b);
        aScored = b;
    }

    private void setLeftLollipopFirst(boolean b){
        System.out.println("left lollipop first SET TO: " + b);
        leftFirst = b;
    }

    @Override
    public void init() {
        // setup
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.AB)));
        addCommands(new InstantCommand(() -> setAScored(true)));
        addCommands(
            new ConditionalCommand(
                new InstantCommand(() -> setLeftLollipopFirst(true)),
                new InstantCommand(() -> setLeftLollipopFirst(false)), 
                 () -> autoFactory.getLeftLollipopFirst())
        );

        // home, then raise to L3 on your way to B
        addCommands(new ParallelCommandGroup(
                followPathCommand(startPath.getChoreoPath()),
                Commands.sequence(
                        toPosition(TargetAction.HOME),
                        Commands.waitUntil(
                                () -> !ElevatorSubsystem.getInstance().isHoming()),
                        new WaitCommand(0.3),
                        toPosition(TargetAction.L3))));

        // align and score preload
        addCommands(Commands.sequence(
                Commands.parallel(
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB)
                                .withTimeout(2.25),
                        toPosition(TargetAction.L4)),
                score(TargetAction.L4)));

        addCommands(
                toPosition(TargetAction.INTAKE),
                ((followPathCommand(loadCenter.getChoreoPath()).beforeStarting(new WaitCommand(0.3)))
                                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake()))));
                        
       // score 1st pickup
        addCommands(new ConditionalCommand(
                Commands.sequence(
                        Commands.parallel(
                                AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB)
                                        .withTimeout(2.25),
                                toPosition(TargetAction.L4)),
                        score(TargetAction.L4)),
                new PrintCommand("DIDN'T GET CENTER").andThen(new InstantCommand(() -> setAScored(false))),
                haveCoral()));

        // 2nd pickup
        addCommands(
                toPosition(TargetAction.INTAKE),
                ((followPathCommand(loadRight.getChoreoPath()).beforeStarting(new WaitCommand(0.2)))
                                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
        );

        // score 2nd pickup - if have coral and L4 not scored, score L4
        addCommands(
            new ConditionalCommand(
                new ConditionalCommand(
                    Commands.sequence(
                        Commands.parallel(
                            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                            toPosition(TargetAction.L2)),
                        score(TargetAction.L2)), 
                    Commands.sequence(
                        Commands.parallel(
                            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                            toPosition(TargetAction.L4)),
                        score(TargetAction.L4),
                        new InstantCommand(() -> setAScored(true))), 
                    () -> aScored),
            new PrintCommand("DIDN'T GET 1st LOLLIPOP"),
            haveCoral()));

        // 3rd pickup
        addCommands(
                toPosition(TargetAction.INTAKE),
                ((followPathCommand(loadLeft.getChoreoPath()).beforeStarting(new WaitCommand(0.2)))
                                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
        );

        // score 3rd pickup - if L4 still hasn't been scored, do it
        addCommands(
            new ConditionalCommand(
                new ConditionalCommand(
                    Commands.sequence(
                        Commands.parallel(
                            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                            toPosition(TargetAction.L2)),
                        score(TargetAction.L2)), 
                    Commands.sequence(
                        Commands.parallel(
                            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB).withTimeout(2.25),
                            toPosition(TargetAction.L4)),
                        score(TargetAction.L4),
                        new InstantCommand(() -> setAScored(true))), 
                    () -> aScored),
            new PrintCommand("DIDN'T GET 2nd LOLLIPOP"),
            haveCoral()));
    }
    // spotless:on
}
