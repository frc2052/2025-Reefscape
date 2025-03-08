// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

/** Add your docs here. */
public class AutoE1D1C1 extends AutoBase {

  private static final PathPlannerPath startingPath = Paths.SR_EF;

  @AutoDescription(description = "Right Side L1 Auto")
  public AutoE1D1C1() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    addCommands(delaySelectedTime());
    addCommands(getBumpCommand());

    addCommands(startHP());
    addCommands(
        safeReefAlignment(startingPath, AlignOffset.MIDDLE_REEF, FieldElementFace.EF)
            .alongWith(
                prepareForScoreWhenReady(TargetAction.L1H)
                    .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
            .andThen(score(TargetAction.L1H)));

    //
    addCommands(safeStationAlignment(Paths.E2_RL));
    addCommands(HPIntake());
    addCommands(
        followPathCommand(Paths.RL_CD_L1)
            .alongWith(
                prepareForScoreWhenReady(TargetAction.L1H)
                    .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
            .andThen(score(TargetAction.L1H)));

    //
    addCommands(safeStationAlignment(Paths.CD_RL));
    addCommands(HPIntake());
    addCommands(
        followPathCommand(Paths.RL_CD_L1)
            .alongWith(
                prepareForScoreWhenReady(TargetAction.L1H)
                    .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
            .andThen(score(TargetAction.L1H)));
  }
}
