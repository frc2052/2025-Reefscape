// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

/** Add your docs here. */
@AutoDescription(description = "Left Side L1's")
public class AutoJ1K1L1 extends AutoBase {

  public static final PathPlannerPath startingPath = Paths.SL_IJ;

  public AutoJ1K1L1() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() { // test
    addCommands(getBumpCommand());
    addCommands(delaySelectedTime());

    addCommands(
        safeReefAlignment(startingPath, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.IJ)
            .alongWith(
              prepareForScoreWhenReady(TargetAction.L1H)
                  .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
          .andThen(score(TargetAction.L1H)));

    //
    addCommands(safeStationAlignment(Paths.J2_LL));
    addCommands(HPIntake());
    addCommands(
        followPathCommand(Paths.LL_KL_L1)
            .alongWith(
              prepareForScoreWhenReady(TargetAction.L1H)
                  .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
          .andThen(score(TargetAction.L1H)));

    //
    addCommands(safeStationAlignment(Paths.KL_LL));
    addCommands(HPIntake());
    addCommands(
        followPathCommand(Paths.LL_KL_L1)
            .alongWith(
              prepareForScoreWhenReady(TargetAction.L1H)
                  .andThen(HandCommandFactory.motorIn().withTimeout(0.05)))
          .andThen(score(TargetAction.L1H)));
  }
}
