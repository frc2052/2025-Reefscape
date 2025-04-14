// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
public class MiddleH4 extends AutoBase {

    private static final Path centerPath = PathsBase.B_SC_GH;

    public MiddleH4() {
        super(centerPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(delaySelectedTime());

        //
        addCommands(new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.GH))
                .andThen(new ParallelCommandGroup(
                        ArmCommandFactory.intake().withTimeout(1),
                        ClimberCommandFactory.climberDown().withTimeout(0.5),
                        AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                        () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.GH)
                                .withTimeout(5.0)))
                .andThen(new InstantCommand(
                        () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L4)))
                .andThen(new WaitCommand(1.3))
                .andThen(score(TargetAction.L4)));
    }
}
