// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake.middle;

import frc.robot.auto.common.AutoBase;

/** Add your docs here. */
public class BlueG4LeftAlgae extends AutoBase{
    private static final Path startPath = PathsBase.B_SC_G;
    private static final Path descore1 = PathsBase.B_GH_SCORE_TO_DESCORE;
    private static final Path score1 = PathsBase.B_GH_NET;
    private static final Path descore2 = PathsBase.B_NET_IJ;
    private static final Path score2 = PathsBase.B_IJ_NET;
    private static final Path descore3 = PathsBase.B_NET_KL;
    private static final Path score3 = PathsBase.B_KL_NET;

    public BlueG4LeftAlgae(){
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        
    }
}
