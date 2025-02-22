// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.StartRight;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;

/** Add your docs here. */
public class AutoE1D1C1 extends AutoBase {

  private static final PathPlannerPath startingPath = Paths.SR_EF;

  @AutoDescription(description = "L1")
  public AutoE1D1C1() {
    super(startingPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    // addCommands(delaySelectedTime());
    // addCommands(getBumpCommand());

    addCommands(
        new InstantCommand(
            () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HP)));
    addCommands(
        safeReefAlignment(startingPath, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.EF));
    addCommands(toPosAndScore(TargetAction.L1H));
    addCommands(safeStationAlignment(Paths.EF_RL));
    addCommands(HPIntake());
    addCommands(
        safeReefAlignment(Paths.RL_EF, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.EF));
    addCommands(toPosAndScore(TargetAction.L1H));
    addCommands(safeStationAlignment(Paths.EF_RL));
    addCommands(HPIntake());
    addCommands(
        safeReefAlignment(Paths.RL_EF, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.EF));
    addCommands(toPosAndScore(TargetAction.L1H));
  }
}
