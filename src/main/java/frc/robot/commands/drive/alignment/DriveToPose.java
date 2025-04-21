package frc.robot.commands.drive.alignment;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private final double driveP = 1.2;
    private final double driveD = 0.05;
    private final double driveMaxSpeed = 1.25;
    private final double driveMaxAcceleration = 1.75;
    private final double driveTolerance = 0.01;
    private final double ffMinRadius = 0.05;
    private final double ffMaxRadius = 0.1;
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final Supplier<Pose2d> target;

    private DelayedBoolean stoppedMovingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.25);

    private final SwerveRequest.FieldCentricFacingAngle driveChassisSpeeds = new SwerveRequest.FieldCentricFacingAngle()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withHeadingPID(5.0, 0, 0.1);

    private boolean running = false;

    private final ProfiledPIDController driveController;

    private Translation2d lastSetpointTranslation = new Translation2d();
    private double driveErrorAbs = 0.0;
    private Supplier<Pose2d> robot = RobotState.getInstance()::getFieldToRobot;

    private DoubleSupplier xlinearFF = () -> 0.0;
    private DoubleSupplier ylinearFF = () -> 0.0;

    private Pose2d targetPose;

    public DriveToPose(Supplier<Pose2d> target) {
        this.target = target;

        driveController = new ProfiledPIDController(
                driveP, 0.0, driveD, new TrapezoidProfile.Constraints(driveMaxSpeed, driveMaxAcceleration), 0.02);
        driveController.setTolerance(driveTolerance);
        addRequirements(drivetrain);
    }

    public DriveToPose(Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        this(target);
        this.robot = robot;
    }

    public DriveToPose(
            Supplier<Pose2d> target, Supplier<Pose2d> robot, DoubleSupplier xlinearFF, DoubleSupplier ylinearFF) {
        this(target, robot);
        this.xlinearFF = xlinearFF;
        this.ylinearFF = ylinearFF;
    }

    @Override
    public void initialize() {
        Logger.recordOutput("DriveToPose/At Goal", false);
        Pose2d currentPose = robot.get();
        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getCurrentRobotChassisSpeeds(), currentPose.getRotation());
        Translation2d linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                currentPose.getTranslation().getDistance(target.get().getTranslation()),
                Math.min(
                        0.0,
                        -linearFieldVelocity
                                .rotateBy(target.get()
                                        .getTranslation()
                                        .minus(currentPose.getTranslation())
                                        .getAngle()
                                        .unaryMinus())
                                .getX()));
        lastSetpointTranslation = currentPose.getTranslation();
        targetPose = null;
    }

    @Override
    public void execute() {
        running = true;
        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        if (target.get() != null) {
            targetPose = target.get();
        }

        if (targetPose == null) {
            System.out.println(this.getName() + " has no target pose");
            return;
        }

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScaler + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                        targetPose.getTranslation(),
                        currentPose
                                .getTranslation()
                                .minus(targetPose.getTranslation())
                                .getAngle())
                .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0, new Rotation2d()))
                .getTranslation();

        Translation2d driveVelocity = new Pose2d(
                        new Translation2d(),
                        currentPose
                                .getTranslation()
                                .minus(targetPose.getTranslation())
                                .getAngle())
                .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
                .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = Math.hypot(xlinearFF.getAsDouble(), ylinearFF.getAsDouble()) * 3.0;
        driveVelocity = driveVelocity.interpolate(
                new Translation2d(xlinearFF.getAsDouble(), ylinearFF.getAsDouble())
                        .times(DrivetrainConstants.DRIVE_MAX_SPEED.in(MetersPerSecond)),
                linearS);

        if (RobotState.getInstance().isRedAlliance()) {
            drivetrain.setControl(driveChassisSpeeds
                    .withTargetDirection(targetPose.getRotation().plus(Rotation2d.k180deg))
                    .withVelocityX(-driveVelocity.getX())
                    .withVelocityY(-driveVelocity.getY()));
        } else {
            drivetrain.setControl(driveChassisSpeeds
                    .withTargetDirection(targetPose.getRotation())
                    .withVelocityX(driveVelocity.getX())
                    .withVelocityY(driveVelocity.getY()));
        }
        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
        Logger.recordOutput(
                "DriveToPose/Setpoint", new Pose2d[] {new Pose2d(lastSetpointTranslation, targetPose.getRotation())});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
        RobotState.getInstance().setIsAtAlignGoal(atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        running = false;
        stoppedMovingDelay.update(Timer.getFPGATimestamp(), false);
        // Clear logs
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/At Goal", true);
        System.out.println("Drive to Pose is complete********");
        if (interrupted) {
            System.out.println("Drive to pose was interuppted");
        }
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    /** Checks if the robot is stopped at the final pose. */
    public boolean atGoal() {
        boolean stoppedMoving = stoppedMovingDelay.update(
                Timer.getFPGATimestamp(),
                MathHelpers.epsilonEquals(
                        MathHelpers.chassisSpeedsNorm(RobotState.getInstance().getChassisSpeeds()), 0.0, 0.05));
        return running && stoppedMoving;
        // return running && driveController.atGoal();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running && Math.abs(driveErrorAbs) < driveTolerance;
    }
}
