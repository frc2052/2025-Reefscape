// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.smartdrive;

import com.team2052.lib.vision.limelight.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.LeftLimelightConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldLocation;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.drive.alignment.DriveToPose;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeAndScoreCommand extends Command {
    SuperstructureSubsystem superstructure;
    private boolean done;

    @SuppressWarnings("unchecked") // TODO: make sure the part below this works as intended.
    private Supplier<Pose2d> target = (Supplier<Pose2d>)
            new Pose2d(RobotState.getInstance().getFieldToRobot().getX(), 4, new Rotation2d(8));
    /** Creates a new AlgaeIntakeAndScoreCommand. */
    public AlgaeIntakeAndScoreCommand(SuperstructureSubsystem superStrucure) {
        this.superstructure = superStrucure;
        addRequirements(superStrucure);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
        AlgaeIntakeAndScore();
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }

    private void AlgaeIntakeAndScore() {
        FieldElementFace face;
        if (RobotState.getFieldLocation() != FieldLocation.BARGE) {
            face = AlignmentCommandFactory.idToReefFace(
                    (int) LimelightHelpers.getFiducialID(LeftLimelightConstants.CAMERA_NAME));
            if (face == FieldElementFace.AB || face == FieldElementFace.EF || face == FieldElementFace.IJ) {
                superstructure.setCurrentAction(TargetAction.UPPER_ALGAE);
            } else {
                superstructure.setCurrentAction(TargetAction.LOWER_ALGAE);
            }
            done = true;
        } else {
            new DriveToPose(target);
            done = true;
        }
    }
}
