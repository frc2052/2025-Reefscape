// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.auto.common.AutoBase;
import frc.robot.auto.common.AutoDescription;
// import frc.robot.commands.drive.AlignWithReefCommand.AlignLocation;
// import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocation;
import frc.robot.controlboard.PositionSuperstructure.ReefSubSide;
import frc.robot.controlboard.PositionSuperstructure.TargetFieldLocation;


@AutoDescription(description = "testing vision to reef side")
public class AutoLLToK4 extends AutoBase {
  private static final PathPlannerPath startPath = Paths.LL_STOP;

  /** Creates a new AutoLLToK4. */
  public AutoLLToK4() {
    super(startPath.getStartingHolonomicPose());
  }

  @Override
  public void init() {
    System.out.println("START LL - K4");
    addCommands(reefSideVisionOrPathAlign(ReefSubSide.LEFT, startPath, TargetFieldLocation.KL));
  }
}
