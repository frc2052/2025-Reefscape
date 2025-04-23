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
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class LeftLolipop extends AutoBase {
    
    private static final Path startPath = PathsBase.SL_A;
    private static final Path loadCenter = PathsBase.AB_LOLIPOP_C;
    private static final Path loadLeft = PathsBase.AB_LOLIPOP_L;
    private static final Path loadRight = PathsBase.AB_LOLIPOP_R;
    
    public LeftLolipop(){
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        // setup
        addCommands(getBumpCommand());
        addCommands(delaySelectedTime());
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.AB)));

        // home, then raise to L3 on your way to B
        addCommands(
            new ParallelCommandGroup(
                followPathCommand(startPath.getChoreoPath()),
                Commands.sequence(
                    toPos(TargetAction.HOME),
                    Commands.waitUntil(() -> !ElevatorSubsystem.getInstance().isHoming()),
                    new WaitCommand(0.3),
                    toPos(TargetAction.L3))
            )
        );

        // align and score preload
        addCommands(
            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB)
            .alongWith(toPos(TargetAction.L4).withTimeout(2.25))
            .andThen(score(TargetAction.L4))
        );

        addCommands(
            // intake pos, short delay to follow path
            Commands.parallel(
                followPathCommand(loadCenter.getChoreoPath())
                .beforeStarting(new WaitCommand(0.3))
                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())),
                toPos(TargetAction.INTAKE)
            )
        );

        // score 1st pickup
        addCommands(
            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB)
            .alongWith(toPos(TargetAction.L4).withTimeout(2.25))
            .andThen(score(TargetAction.L4))
        );

        // 2nd pickup
        addCommands(
            // intake pos, short delay to follow path
            Commands.parallel(
                followPathCommand(loadLeft.getChoreoPath())
                .beforeStarting(new WaitCommand(0.3))
                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())),
                toPos(TargetAction.INTAKE)
            )
        );

        // score 2nd pickup
        addCommands(
            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB)
            .alongWith(toPos(TargetAction.L2).withTimeout(2.25))
            .andThen(score(TargetAction.L2))
        );

        // 3rd pickup
        addCommands(
            // intake pos, short delay to follow path
            Commands.parallel(
                followPathCommand(loadRight.getChoreoPath())
                .beforeStarting(new WaitCommand(0.3))
                .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())),
                toPos(TargetAction.INTAKE)
            )
        );

        // score 3rd pickup
        addCommands(
            AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB)
            .alongWith(toPos(TargetAction.L2).withTimeout(2.25))
            .andThen(score(TargetAction.L2))
        );
    }
}
