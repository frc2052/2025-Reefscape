// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;

public class AlgaePivotSubsystem extends SubsystemBase {
    private static AlgaePivotSubsystem INSTANCE;
    private boolean manualPctMode;

    private final TalonFX pivotMotor;

    private Angle goalPosition;

    public static AlgaePivotSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AlgaePivotSubsystem();
        }
        return INSTANCE;
    }

    private AlgaePivotSubsystem() {
        pivotMotor = new TalonFX(Ports.ALGAE_PIVOT_ID);

        pivotMotor.getConfigurator().apply(AlgaePivotConstants.MOTOR_CONFIG);

        manualPctMode = false;
        goalPosition = Degrees.of(135);
    }

    private void updatePivot() {
        if (Math.abs(getPivotAngle().in(Degrees) - goalPosition.in(Degrees)) > 40) {
            pivotMotor.set(Math.copySign(0.4, -(getPivotAngle().in(Degrees) - goalPosition.in(Degrees))));
        } else if (Math.abs(getPivotAngle().in(Degrees) - goalPosition.in(Degrees)) > 20) {
            pivotMotor.set(Math.copySign(0.25, -(getPivotAngle().in(Degrees) - goalPosition.in(Degrees))));
        } else if (Math.abs(getPivotAngle().in(Degrees) - goalPosition.in(Degrees)) > 2) {
            pivotMotor.set(Math.copySign(0.15, -(getPivotAngle().in(Degrees) - goalPosition.in(Degrees))));
        } else {
            pivotMotor.set(0.0);
        }
    }

    public void setPivotAngle(Angle angle) {
        manualPctMode = false;
        goalPosition = angle;
        if (angle == goalPosition && isAtDesiredPosition()) {
            return;
        }

        // pivotMotor.setControl(new PositionTorqueCurrentFOC(angle).withFeedForward(Amps.of(19)));
    }

    public void setGoalPosition(TargetAction position) {
        this.goalPosition = position.getAlgaeArmPivotPosition();
        // setPivotAngle(goalPosition);
    }

    public Command runPivotPct(double pct) {
        return Commands.runOnce(() -> setPivotSpeedManual(pct), this);
    }

    public void setPivotSpeedManual(double pct) {
        manualPctMode = true;
        pivotMotor.set(pct);
    }

    public Angle getPivotDesiredPosition() {
        return goalPosition;
    }

    public Angle getPivotAngle() {
        return pivotMotor.getPosition().getValue();
    }

    public boolean isAtDesiredPosition() {
        return isAtDesiredPosition(AlgaePivotConstants.TOLERANCE);
    }

    public boolean isAtDesiredPosition(double tol) {
        return isAtPosition(tol, goalPosition);
    }

    public boolean isAtPosition(double tol, Angle goal) {
        return Math.abs(getPivotPosition().in(Degrees) - goal.in(Degrees)) <= tol;
    }

    public Angle getPivotPosition() {
        return Rotations.of(pivotMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (!manualPctMode) {
            updatePivot();
        }
    }
}
