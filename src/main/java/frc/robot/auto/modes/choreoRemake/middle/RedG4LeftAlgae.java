// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake.middle;

import frc.robot.auto.common.AutoBase;

/** Add your docs here. */
public class RedG4LeftAlgae extends AutoBase {
    private static final Path startPath = PathsBase.R_SC_GH;
    private static final Path descore1 = PathsBase.R_GH_REPOSITION;
    private static final Path score1 = PathsBase.R_GH_NET;
    private static final Path descore2 = PathsBase.R_NET_IJ;
    private static final Path score2 = PathsBase.R_IJ_NET;
    private static final Path descore3 = PathsBase.R_NET_KL;
    private static final Path score3 = PathsBase.R_KL_NET;

    public RedG4LeftAlgae() {
        super(startPath.getChoreoPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {}
}
