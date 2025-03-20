// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private CANcoder encoder;
    private Angle goalPosition;

    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final ArmFeedforward ff;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /** Public method to provide access to the instance. */
    private static IntakePivotSubsystem INSTANCE;

    public static IntakePivotSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakePivotSubsystem();
        }
        return INSTANCE;
    }

    private IntakePivotSubsystem() {
        goalPosition = Degrees.of(30); // TargetAction.STOW.getIntakePivotPosition();
        pivotMotor = new TalonFX(Ports.INTAKE_PIVOT_ID);
        pivotMotor.getConfigurator().apply(IntakePivotConstants.MOTOR_CONFIG);

        encoder = new CANcoder(Ports.INTAKE_ENCODER_ID);
        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoderConfig.MagnetSensor.MagnetOffset = IntakePivotConstants.ENCODER_OFFSET.in(Rotations);
        encoder.getConfigurator().apply(armEncoderConfig, 1.0);

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                IntakePivotConstants.MAX_VELOCITY, IntakePivotConstants.MAX_ACCELERATION));
        ff = new ArmFeedforward(
                IntakePivotConstants.kS, IntakePivotConstants.kG, IntakePivotConstants.kV, IntakePivotConstants.kA);
    }

    public void runPivot(Angle setpoint, double feedforward) {
        // pivotMotor.setControl(
        //         positionControl.withPosition(setpoint.in(Rotations)).withFeedForward(feedforward));
    }

    public void setPosition(TargetAction action) {
        goalPosition = action.getIntakePivotPosition();
    }

    public boolean isAtDesiredPosition() {
        return isAtDesiredPosition(IntakePivotConstants.DEG_TOL);
    }

    public boolean isAtDesiredPosition(double tol) {
        return isAtPosition(tol, goalPosition);
    }

    public boolean isAtPosition(double tol, Angle goal) {
        return Math.abs(getPosition().in(Degrees) - goal.in(Degrees)) <= tol;
    }

    public Angle getPosition() {
        return Rotations.of(pivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake Pivot/Angle", getPosition().in(Degrees));
        Logger.recordOutput("Intake Pivot/Goal Angle", goalPosition.in(Degrees));

        // if (DriverStation.isDisabled()) {
        //     pivotMotor.stopMotor();
        //     // Reset profile when disabled
        //     setpoint = new TrapezoidProfile.State(getPosition().in(Rotations), 0);
        // }

        // if (!DriverStation.isDisabled()) {
        //     setpoint = profile.calculate(
        //             0.02,
        //             setpoint,
        //             new TrapezoidProfile.State(
        //                     MathUtil.clamp(
        //                             goalPosition.in(Rotations),
        //                             IntakePivotConstants.MIN_INTAKE_ARM_ANGLE.in(Rotations),
        //                             IntakePivotConstants.MAX_INTAKE_ARM_ANGLE.in(Rotations)),
        //                     0.0));
        // } else {
        //     runPivot(Rotations.of(setpoint.position), ff.calculate(setpoint.position, setpoint.velocity));
        // }
    }

    public void voltageDrive(Voltage voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }

    SysIdRoutine routine =
            new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::voltageDrive, null, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
