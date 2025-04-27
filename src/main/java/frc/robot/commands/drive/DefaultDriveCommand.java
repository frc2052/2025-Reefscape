// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {

    protected final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldCentricSupplier;

    protected final double maxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double maxAngularRate =
            RotationsPerSecond.of(0.6).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    protected final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

    protected final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);
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

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    protected double getX() {
        return xSupplier.getAsDouble();
    }

    protected double getY() {
        return ySupplier.getAsDouble();
    }

    protected double getRotation() {
        return rotationSupplier.getAsDouble();
    }

    protected boolean getFieldCentric() {
        return fieldCentricSupplier.getAsBoolean();
    }

    protected SwerveRequest getSwerveRequest() {
        if (getFieldCentric()) {
            return fieldCentricDrive
                    .withVelocityX(getX() * maxSpeed)
                    .withVelocityY(getY() * maxSpeed)
                    .withRotationalRate(getRotation() * maxAngularRate);
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
}
