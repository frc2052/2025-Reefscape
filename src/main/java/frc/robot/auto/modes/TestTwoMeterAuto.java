// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.auto.AutoBase;
import frc.robot.auto.AutoBase.Paths;

/** Add your docs here. */
public class TestTwoMeterAuto extends AutoBase{

    public TestTwoMeterAuto(){
        super(getStartPoseFromAutoFile("Test Auto"));
    }

    @Override
    public void init() {
        waitTime();
        addCommands(followPathCommand(Paths.test2MeterPath));
    }
}
