// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.auto.common.AutoBase;


/** Add your docs here. */
public class TestTwoMeterAuto extends AutoBase{

    public TestTwoMeterAuto(){
        super(getStartPoseFromAutoFile("Test Auto"));
    }

    @Override
    public void init() {
        delaySelectedTime();
        AutoBuilder.buildAuto("Test Auto - 2 Meters Straight");
        // addCommands(followPathCommand(Paths.TEST_PATH_2_METERS));
    }
}
