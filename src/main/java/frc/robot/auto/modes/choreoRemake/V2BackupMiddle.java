// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.choreoRemake;

import frc.robot.auto.common.AutoBase;
import frc.robot.commands.intake.IntakeCommandFactory;

/** Add your docs here. */
public class V2BackupMiddle extends AutoBase {

    private static final Path startPath = PathsBase.SC_GH_L1;

    public V2BackupMiddle() {
        super(startPath.getPathPlannerPath().getStartingHolonomicPose());
    }

    @Override
    public void init() {
        addCommands(getBumpCommand());

        addCommands(followPathCommand(startPath.getPathPlannerPath()));

        addCommands(IntakeCommandFactory.outtake().withTimeout(3.0));
    }
}
