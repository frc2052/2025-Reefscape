package frc.robot.commands.smartdrive;

import com.team2052.lib.vision.limelight.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants.LeftLimelightConstants;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.drive.alignment.DriveToPose;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.FieldElementFace;

public class AlgaeIntakeAndScore {

    public static Command algaeIntakeCommand() {
        FieldElementFace face;
        face = AlignmentCommandFactory.idToReefFace(
                (int) LimelightHelpers.getFiducialID(LeftLimelightConstants.CAMERA_NAME));
        if (face == FieldElementFace.AB || face == FieldElementFace.EF || face == FieldElementFace.IJ) {
            return Commands.runOnce(
                    () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.UA),
                    SuperstructureSubsystem.getInstance());
        } else {
            return Commands.runOnce(
                    () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.LA),
                    SuperstructureSubsystem.getInstance());
        }
    }

    public static Command algaeScoreCommand() {
        SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS);
        return new DriveToPose(null);
    }
}
