// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {

    protected final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldCentricSupplier;

    // private final SlewRateLimiter xLimiter;
    // private final SlewRateLimiter yLimiter;
    // private final SlewRateLimiter rotationLimiter;

    protected final double maxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double maxAngularRate =
            RotationsPerSecond.of(0.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private double joystickLastTouched = -1;

    protected final SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05) // Add a 5% deadband in open loop
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    protected final SwerveRequest.ApplyFieldSpeeds driveChassisSpeeds = new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    protected final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);

    protected SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    /**
     * @param xSupplier supplier for forward velocity.
     * @param ySupplier supplier for sideways velocity.
     * @param rotationSupplier supplier for angular velocity.
     */
    public DefaultDriveCommand(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldCentricSupplier) {

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;

        driveWithHeading.HeadingController.setPID(2.5, 0.0, 0.0);
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // xLimiter = new SlewRateLimiter(3.5);
        // yLimiter = new SlewRateLimiter(3.5);
        // rotationLimiter = new SlewRateLimiter(5);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingSetpoint = Optional.empty();
    }

    protected double getX() {
        return xSupplier.getAsDouble();
        // if (Math.abs(xSupplier.getAsDouble()) > DriverConstants.GAMEPAD_DEADBAND) {
        //   return slewAxis(xLimiter, expo(xSupplier.getAsDouble()) * maxSpeed);
        // }

        // return 0.0;
    }

    protected double getY() {
        return ySupplier.getAsDouble();
        // if (Math.abs(ySupplier.getAsDouble()) > DriverConstants.GAMEPAD_DEADBAND) {
        //   return slewAxis(yLimiter, expo(ySupplier.getAsDouble()) * maxSpeed);
        // }

        // return 0.0;
    }

    protected double getRotation() {
        return rotationSupplier.getAsDouble();
        // if (Math.abs(rotationSupplier.getAsDouble()) > DriverConstants.GAMEPAD_DEADBAND) {
        //   return slewAxis(rotationLimiter, expo(rotationSupplier.getAsDouble()) * maxAngularRate);
        // }
        // return 0.0;
    }

    protected boolean getFieldCentric() {
        return fieldCentricSupplier.getAsBoolean();
    }

    protected SwerveRequest getSwerveRequest() {
        if (getFieldCentric()) {
            // if (Math.abs(getRotation()) > DriverConstants.STEER_DEADBAND) {
            //   joystickLastTouched = Timer.getFPGATimestamp();
            // }

            // if (Math.abs(getRotation()) > DriverConstants.STEER_DEADBAND
            //     || (MathHelpers.epsilonEquals(joystickLastTouched, Timer.getFPGATimestamp(), 0.25)
            //         && Math.abs(drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)
            //             > Math.toRadians(10))) {
            headingSetpoint = Optional.empty();
            return driveNoHeading
                    .withVelocityX(getX() * maxSpeed)
                    .withVelocityY(getY() * maxSpeed)
                    .withRotationalRate(getRotation() * maxAngularRate);
            // } else {
            //   if (headingSetpoint.isEmpty()) {
            //     headingSetpoint = Optional.of(drivetrain.getState().Pose.getRotation());
            //     if (RobotState.getInstance().isRedAlliance()) {
            //       headingSetpoint.get().rotateBy(Rotation2d.k180deg);
            //     }
            //   }
            //   Logger.recordOutput("DefaultDriveCommand/Mode", "Heading");
            //   Logger.recordOutput(
            //       "DefaultDriveCommand/HeadingSetpoint", headingSetpoint.get().getDegrees());

            //   return driveWithHeading
            //       .withVelocityX(getX() * maxSpeed)
            //       .withVelocityY(getY() * maxSpeed)
            //       .withTargetDirection(headingSetpoint.get());
            // }
        } else {
            return robotCentricDrive.withVelocityX(getX()).withVelocityY(getY()).withRotationalRate(getRotation());
        }
    }

    @Override
    public void execute() {
        drivetrain.setControl(getSwerveRequest());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // protected double expo(double value) {
    //   if (DriverConstants.DEV_CONTROLS) {
    //     return Math.copySign(Math.pow(Math.abs(value), 2), value);
    //   }

    //   return Math.copySign(Math.pow(Math.abs(value), 1.5), value);
    // }

    // protected double slewAxis(SlewRateLimiter limiter, double value) {
    //   value = limiter.calculate(value);

    //   return value;
    // }
}
