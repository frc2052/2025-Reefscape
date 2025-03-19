// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private Angle goalPosition;

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
    }

    public void setAngle(Angle angle) {
        if (angle == goalPosition && isAtDesiredPosition()) {
            return;
        }
        System.out.println(angle.in(Degrees));
        pivotMotor.setControl(new MotionMagicVoltage(angle.in(Rotations)));
    }

    public void setPosition(TargetAction action) {
        setAngle(action.getIntakePivotPosition());
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
    }
}
