package frc.robot.commands.drive.alignment;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final double driveP = 4.0;
  private final double driveD = 0.0;
  private final double turnP = 3.5;
  private final double turnD = 0.0;
  private final double driveMaxSpeed = 3.8;
  private final double driveMaxAcceleration = 3.0;
  private final double turnMaxSpeed = Units.degreesToRadians(540.0);
  private final double turnMaxAcceleration = 8.0;
  private final double driveTolerance = 0.04;
  private final double turnTolerance = Units.degreesToRadians(0.5);
  private final double ffMinRadius = 0.1;
  private final double ffMaxRadius = 0.15;
  private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  private final Supplier<Pose2d> target;

  private final SwerveRequest.ApplyRobotSpeeds driveChassisSpeeds =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDesaturateWheelSpeeds(true)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private boolean running = false;

  private final ProfiledPIDController driveController;
  private final ProfiledPIDController turnController;

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private Supplier<Pose2d> robot = RobotState.getInstance()::getFieldToRobot;

  private DoubleSupplier xlinearFF = () -> 0.0;
  private DoubleSupplier ylinearFF = () -> 0.0;
  private DoubleSupplier omegaFF = () -> 0.0;

  private Pose2d targetPose;

  public DriveToPose(Supplier<Pose2d> target) {
    this.target = target;

    driveController =
        new ProfiledPIDController(
            driveP,
            0.0,
            driveD,
            new TrapezoidProfile.Constraints(driveMaxSpeed, driveMaxAcceleration),
            0.02);
    driveController.setTolerance(driveTolerance);

    turnController =
        new ProfiledPIDController(
            turnP,
            0.0,
            turnD,
            new TrapezoidProfile.Constraints(turnMaxSpeed, turnMaxAcceleration),
            0.02);
    turnController.setTolerance(turnTolerance);

    // Enable continuous input for theta controller
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  public DriveToPose(Supplier<Pose2d> target, Supplier<Pose2d> robot) {
    this(target);
    this.robot = robot;
  }

  public DriveToPose(
      Supplier<Pose2d> target,
      Supplier<Pose2d> robot,
      DoubleSupplier xlinearFF,
      DoubleSupplier ylinearFF,
      DoubleSupplier omegaFF) {
    this(target, robot);
    this.xlinearFF = xlinearFF;
    this.ylinearFF = ylinearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = robot.get();
    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getCurrentRobotChassisSpeeds(), currentPose.getRotation());
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVelocity
                .rotateBy(
                    target
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    turnController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
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
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(driveController.getSetpoint().position, 0.0, new Rotation2d()))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        turnController.getSetpoint().velocity * ffScaler
            + turnController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < turnController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    // Scale feedback velocities by input ff
    final double linearS = Math.hypot(xlinearFF.getAsDouble(), ylinearFF.getAsDouble()) * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(
            new Translation2d(xlinearFF.getAsDouble(), ylinearFF.getAsDouble())
                .times(DrivetrainConstants.DRIVE_MAX_SPEED.in(MetersPerSecond)),
            linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity,
            omegaFF.getAsDouble() * DrivetrainConstants.DRIVE_MAX_ANGULAR_RATE.in(RadiansPerSecond),
            thetaS);

    // Command speeds
    drivetrain.setControl(
        driveChassisSpeeds.withSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                currentPose.getRotation())));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", turnController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(turnController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && turnController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
