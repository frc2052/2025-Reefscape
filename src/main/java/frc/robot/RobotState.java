package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState.FieldLocation;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private SwerveDriveState drivetrainState = new SwerveDriveState();
    private AlignOffset selectedAlignOffset = AlignOffset.MIDDLE_REEF;
    private Pose2d autoStartPose;
    private FieldElementFace seenReefFace;
    private FieldElementFace desiredReefFace;
    private boolean isAlignGoal;
    private boolean isIntaking;
    private static FieldLocation loca;

    private boolean isReefTracking;
    private boolean hasCoral;
    private boolean isFlushAlign;

    private static RobotState INSTANCE;

    public static RobotState getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new RobotState();
        }

        return INSTANCE;
    }

    private RobotState() {}

    public void setIsIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public boolean getIsIntaking() {
        return isIntaking;
    }

    public void setFieldLocation(FieldLocation location) {
        loca = location;
    }

    public static FieldLocation getFieldLocation() {
        return loca;
    }

    public ChassisSpeeds getChassisSpeeds(boolean isFieldRelative) {
        ChassisSpeeds chassisSpeeds = drivetrainState.Speeds;
        if (isFieldRelative) {
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                    chassisSpeeds, getFieldToRobot().getRotation());
        } else {
            return chassisSpeeds;
        }
    }

    public void setReefTracking(boolean isReefTracking) {
        this.isReefTracking = isReefTracking;
    }

    public boolean getIsReefTracking() {
        return isReefTracking;
    }

    public void setIsFlushAlign(boolean isFlushAlign) {
        this.isFlushAlign = isFlushAlign;
    }

    public boolean getIsFlushAlign() {
        return isFlushAlign;
    }

    public void setHasCoral(boolean hasCoral) {
        this.hasCoral = hasCoral;
    }

    public boolean getHasCoral() {
        return hasCoral;
    }

    public Pose2d getFieldToRobot() {
        if (drivetrainState.Pose != null) {
            return drivetrainState.Pose;
        }

        return MathHelpers.POSE_2D_ZERO;
    }

    public void addDrivetrainState(SwerveDriveState drivetrainState) {
        this.drivetrainState = drivetrainState;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrainState.Speeds;
    }

    public double getRotationalSpeeds() {
        return drivetrainState.Speeds.omegaRadiansPerSecond;
    }

    public double distanceToAlignPose() {
        if (getAlignPose() == null) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.abs(
                getAlignPose().getTranslation().getDistance(getFieldToRobot().getTranslation()));
    }

    public Command setAlignOffsetCommand(AlignOffset offset) {
        return new InstantCommand(() -> setAlignOffset(offset));
    }

    public void setAlignOffset(AlignOffset offset) {
        // System.out.println("NEW OFFSET " + offset.toString());
        selectedAlignOffset = offset;
    }

    public AlignOffset getAlignOffset() {
        return selectedAlignOffset;
    }

    public void seenReefFaceID(int tagID) {
        seenReefFace = AlignmentCommandFactory.idToReefFace(tagID);
    }

    public void setDesiredReefFace(FieldElementFace reefFace) {
        desiredReefFace = reefFace;
    }

    public boolean shouldAlign(double dist) { // only used in auto
        return getFieldToRobot()
                        .getTranslation()
                        .getDistance(isRedAlliance() ? FieldConstants.RED_REEF_CENTER : FieldConstants.BLUE_REEF_CENTER)
                < dist;
    }

    public boolean shouldAlignAutonomous(double dist) {
        return desiredReefFaceIsSeen() && shouldAlign(dist);
    }

    public Pose2d getAlignPose() {
        // if (selectedAlignOffset == AlignOffset.MIDDLE_REEF) {
        //     return getFieldToRobot()
        //             .nearest(isRedAlliance() ? FieldConstants.redLeftBranchL1 : FieldConstants.blueLeftBranchL1);
        // } else
        if (selectedAlignOffset == AlignOffset.LEFT_BRANCH) {
            return getFieldToRobot()
                    .nearest(isRedAlliance() ? FieldConstants.redLeftBranches : FieldConstants.blueLeftBranches);
        } else if (selectedAlignOffset == AlignOffset.RIGHT_BRANCH) {
            return getFieldToRobot()
                    .nearest(isRedAlliance() ? FieldConstants.redRightBranches : FieldConstants.blueRightBranches);
        } else {
            return getFieldToRobot(); // this should never happen
        }
    }

    public boolean desiredReefFaceIsSeen() {
        if (seenReefFace == null || desiredReefFace == null) {
            return false;
        }

        return desiredReefFace.getTagID() == seenReefFace.getTagID();
    }

    public void setAutoStartPose(Pose2d startPose) {
        this.autoStartPose = startPose;
    }

    public boolean getisAlignGoal() {
        return isAlignGoal;
    }

    public void setIsAtAlignGoal(boolean atGoal) {
        isAlignGoal = atGoal;
    }
    /**
     * Returns true if the robot is on red alliance.
     *
     * @return True if the robot is on red alliance.
     */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red) ? true : false;
        } else {
            return false;
        }
    }

    public void output() {
        Logger.recordOutput("HAS CORAL", hasCoral);
        Logger.recordOutput("Swerve Module States", drivetrainState.ModuleStates);
        Logger.recordOutput("Swerve Module Goals", drivetrainState.ModuleTargets);
        Logger.recordOutput("Current Pose", drivetrainState.Pose);
        Logger.recordOutput("Auto Start Pose", autoStartPose);
        Logger.recordOutput("Goal Align Pose", getAlignPose());
        Logger.recordOutput(
                "Goal Left Alignment",
                getFieldToRobot()
                        .nearest(isRedAlliance() ? FieldConstants.redLeftBranches : FieldConstants.blueLeftBranches));
        Logger.recordOutput(
                "Goal Right Alignment",
                getFieldToRobot()
                        .nearest(isRedAlliance() ? FieldConstants.redRightBranches : FieldConstants.blueRightBranches));
        Logger.recordOutput("Flush Alignment", isFlushAlign);
    }

    public enum FieldLocation {
        REEF(new Area2D(Constants.SmartDrive.blueReef), new Area2D(Constants.SmartDrive.blueReef)),
        BARGE(new Area2D(Constants.SmartDrive.blueBarge), new Area2D(Constants.SmartDrive.redBarge)),
        PROCESSOR(new Area2D(Constants.SmartDrive.blueProcessor), new Area2D(Constants.SmartDrive.redProcessor)),
        TRAVEL(new Area2D(Constants.SmartDrive.blueTravel), new Area2D(Constants.SmartDrive.redTravel));

        public final Area2D blueArea;
        public final Area2D redArea;

        private FieldLocation(Area2D blueArea, Area2D redArea) {
            this.blueArea = blueArea;
            this.redArea = redArea;
        }

        public Area2D getArea() {
            if (isRedAlliance()) {
                return redArea;
            } else {
                return blueArea;
            }
        }
    }

    public void run() {
        AdvantageScopeSubsystem.getInstance().periodic();
        Logger.recordOutput("Flush Alignment", isFlushAlign);
    }

    public enum FieldLocation {
        REEF(new Area2D(Constants.SmartDrive.blueReef), new Area2D(Constants.SmartDrive.blueReef)),
        BARGE(new Area2D(Constants.SmartDrive.blueBarge), new Area2D(Constants.SmartDrive.redBarge)),
        PROCESSOR(new Area2D(Constants.SmartDrive.blueProcessor), new Area2D(Constants.SmartDrive.redProcessor)),
        TRAVEL(new Area2D(Constants.SmartDrive.blueTravel), new Area2D(Constants.SmartDrive.redTravel));

        public final Area2D blueArea;
        public final Area2D redArea;

        private FieldLocation(Area2D blueArea, Area2D redArea) {
            this.blueArea = blueArea;
            this.redArea = redArea;
        }

        public Area2D getArea() {
            if (isRedAlliance()) {
                return redArea;
            } else {
                return blueArea;
            }
        }
    }

    public void run() {
        AdvantageScopeSubsystem.getInstance().periodic();
        Logger.recordOutput("Flush Alignment", isFlushAlign);
    }
}
