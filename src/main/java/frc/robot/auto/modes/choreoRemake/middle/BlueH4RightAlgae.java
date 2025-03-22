// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake.middle;

import frc.robot.auto.common.AutoBase;

/** Add your docs here. */
public class BlueH4RightAlgae extends AutoBase{
    private static final Path startPath = PathsBase.B_SC_H;
    private static final Path descore1 = PathsBase.B_GH_SCORE_TO_DESCORE;
    private static final Path score1 = PathsBase.B_GH_NET;
    private static final Path descore2 = PathsBase.B_NET_EF;
    private static final Path score2 = PathsBase.B_EF_NET;
    private static final Path descore3 = PathsBase.B_NET_CD;
    private static final Path score3 = PathsBase.B_CD_NET;

    public BlueH4RightAlgae(){
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        
    }
}
