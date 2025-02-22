// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.StartLeft;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
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
  public void init() {
    // addCommands(delaySelectedTime());
    // addCommands(getBumpCommand());

    addCommands(startHP());
    addCommands(
        new SequentialCommandGroup(
                new InstantCommand(
                    () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L1H)),
                safeReefAlignment(
                    startingPath, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.IJ))
            .alongWith(
                elevatorToPos(TargetAction.L1H).beforeStarting(waitUntilSlowAndCloseEnough())));
    addCommands(toPosAndScore(TargetAction.L1H));
    addCommands(safeStationAlignment(Paths.J2_LL));
    addCommands(HPIntake());
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(
                    () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.L1H))
                .beforeStarting(new WaitCommand(0.5)),
            followPathCommand(Paths.LL_KL_L1)));
    // addCommands(
    //     safeReefAlignment(Paths.LL_KL_L1, AlignOffset.MIDDLE_REEF_LOC, TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L1H));
    addCommands(safeStationAlignment(Paths.KL_LL));
    addCommands(HPIntake());
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new WaitCommand(0.5),
                new InstantCommand(
                        () ->
                            SuperstructureSubsystem.getInstance()
                                .setCurrentAction(TargetAction.L1H))
                    .beforeStarting(new WaitCommand(0.5)),
                followPathCommand(Paths.LL_KL_L1))));
    // addCommands(safeReefAlignment(Paths.LL_KL_L1, AlignOffset.MIDDLE_REEF_LOC,
    // TargetFieldLocation.KL));
    addCommands(toPosAndScore(TargetAction.L1H));
  }
}
