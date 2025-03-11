package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.vision.limelight.LimelightHelpers;
import com.team2052.lib.vision.limelight.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants.LeftLimelightConstants;
import frc.robot.Constants.VisionConstants.RightLimelightConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.FieldConstants;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    private static final double xyStdDevCoefficient = 0.02;
    private static final double thetaStdDevCoefficient = 0.04;
    private final RobotState robotState = RobotState.getInstance();
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    private PoseEstimate previousLeftEstimate;
    private PoseEstimate previousRightEstimate;

    public VisionIOLimelight() {
        configureLimelights();
    }

    private void configureLimelights() {
        // TODO: set camera poses up
        LimelightHelpers.setCameraPose_RobotSpace(
                LeftLimelightConstants.CAMERA_NAME,
                LeftLimelightConstants.X_OFFSET.in(Meters),
                LeftLimelightConstants.Y_OFFSET.in(Meters),
                LeftLimelightConstants.Z_OFFSET.in(Meters),
                LeftLimelightConstants.THETA_X_OFFSET.in(Degrees),
                LeftLimelightConstants.THETA_Y_OFFSET.in(Degrees),
                LeftLimelightConstants.THETA_Z_OFFSET.in(Degrees));
        LimelightHelpers.setCameraPose_RobotSpace(
                RightLimelightConstants.CAMERA_NAME,
                RightLimelightConstants.X_OFFSET.in(Meters),
                RightLimelightConstants.Y_OFFSET.in(Meters),
                RightLimelightConstants.Z_OFFSET.in(Meters),
                RightLimelightConstants.THETA_X_OFFSET.in(Degrees),
                RightLimelightConstants.THETA_Y_OFFSET.in(Degrees),
                RightLimelightConstants.THETA_Z_OFFSET.in(Degrees));

        LimelightHelpers.SetIMUMode(LeftLimelightConstants.CAMERA_NAME, 0);
        LimelightHelpers.SetIMUMode(RightLimelightConstants.CAMERA_NAME, 0);
        // TODO: make sure MT1 and MT2 work before trying different imu modes
    }

    public void update() {
        Optional<PoseEstimate> leftEstimate = pollLL(LeftLimelightConstants.CAMERA_NAME, previousLeftEstimate);
        Optional<PoseEstimate> rightEstimate = pollLL(RightLimelightConstants.CAMERA_NAME, previousRightEstimate);

        double leftStdDev = Double.MAX_VALUE;
        double leftHeadingStdDev = Double.MAX_VALUE;
        double rightStdDev = Double.MAX_VALUE;
        double rightHeadingStdDev = Double.MAX_VALUE;

        if (MathHelpers.chassisSpeedsNorm(robotState.getChassisSpeeds()) < 3.0
                && (!DriverStation.isAutonomous()
                        || (robotState
                                        .getFieldToRobot()
                                        .getTranslation()
                                        .getDistance(
                                                robotState.isRedAlliance()
                                                        ? FieldConstants.RED_REEF_CENTER
                                                        : FieldConstants.BLUE_REEF_CENTER)
                                < 3))) {
            if (leftEstimate.isPresent() && leftEstimate.get().rawFiducials.length > 0) {
                double closestTagDist = Arrays.stream(leftEstimate.get().rawFiducials)
                        .mapToDouble(fiducial -> fiducial.distToCamera)
                        .min()
                        .getAsDouble();
                leftStdDev = xyStdDevCoefficient * Math.pow(closestTagDist, 2) / leftEstimate.get().tagCount;
                leftHeadingStdDev = thetaStdDevCoefficient * Math.pow(closestTagDist, 2) / leftEstimate.get().tagCount;
                if (leftEstimate.get().avgTagDist > 3.5) leftStdDev = Double.MAX_VALUE;
            }
            if (rightEstimate.isPresent() && rightEstimate.get().rawFiducials.length > 0) {
                double closestTagDist = Arrays.stream(rightEstimate.get().rawFiducials)
                        .mapToDouble(fiducial -> fiducial.distToCamera)
                        .min()
                        .getAsDouble();
                rightStdDev = xyStdDevCoefficient * Math.pow(closestTagDist, 2) / rightEstimate.get().tagCount;
                rightHeadingStdDev =
                        thetaStdDevCoefficient * Math.pow(closestTagDist, 2) / rightEstimate.get().tagCount;
                if (rightEstimate.get().avgTagDist > 3.5) rightStdDev = Double.MAX_VALUE;
            }
            if (leftStdDev < rightStdDev) {
                drivetrain.addVisionMeasurement(
                        leftEstimate.get().pose,
                        leftEstimate.get().timestampSeconds,
                        VecBuilder.fill(leftStdDev, leftStdDev, leftHeadingStdDev));
                robotState.seenReefFaceID((int) LimelightHelpers.getFiducialID(LeftLimelightConstants.CAMERA_NAME));
            } else if (rightStdDev < leftStdDev) {
                drivetrain.addVisionMeasurement(
                        leftEstimate.get().pose,
                        leftEstimate.get().timestampSeconds,
                        VecBuilder.fill(rightStdDev, rightStdDev, rightHeadingStdDev));
                robotState.seenReefFaceID((int) LimelightHelpers.getFiducialID(RightLimelightConstants.CAMERA_NAME));
            }
        }
    }

    public Optional<PoseEstimate> pollLL(String id, PoseEstimate previousEstimate) {
        LimelightHelpers.SetRobotOrientation(
                id, robotState.getFieldToRobot().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        if (LimelightHelpers.getTV(id)) {
            double oldTimestamp = previousEstimate != null ? previousEstimate.timestampSeconds : Double.MAX_VALUE;
            PoseEstimate newEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(id);
            if (newEstimate != null) {
                Logger.recordOutput(id + " MT2 pose", newEstimate.pose);
                if (newEstimate.timestampSeconds == oldTimestamp) {
                    return Optional.empty(); // no new data
                } else {
                    previousEstimate = newEstimate;
                    return Optional.of(newEstimate);
                }
            }

            PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(id);
            if (mt1Estimate != null) {
                Logger.recordOutput(id + " MT1 pose", mt1Estimate.pose);
            }
        }
        return Optional.empty();
    }

    public boolean hasReefTarget() {
        return false;
    }
}
