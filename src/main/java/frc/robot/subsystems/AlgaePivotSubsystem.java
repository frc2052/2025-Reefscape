// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class AlgaePivotSubsystem extends SubsystemBase {
  private static AlgaePivotSubsystem INSTANCE;

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

    goalPosition = TargetAction.HP.getAlgaeArmPivotPosition();
  }

  public void setPivotAngle(Angle angle) {
    if (angle == goalPosition && isAtDesiredPosition()) {
      return;
    }

    pivotMotor.setControl(new PositionVoltage(angle));
  }

  public void setGoalPosition(TargetAction position) {
    this.goalPosition = position.getAlgaeArmPivotPosition();
    // setPivotAngle(goalPosition);
  }

  public Command runPivotPct(double pct) {
    return Commands.runOnce(() -> setPivotSpeed(pct), this);
  }

  private void setPivotSpeed(double pct) {
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
    Logger.recordOutput("Algae Arm/Angle", getPivotPosition().in(Degrees));
    Logger.recordOutput("Algae Arm/Goal Angle", goalPosition.in(Degrees));
    Logger.recordOutput("Algae Arm/At Goal", isAtDesiredPosition());

    // if (isIntaking) {
    //   if (intakingDelay.update(
    //       Timer.getFPGATimestamp(),
    //       MathHelpers.epsilonEquals(pivotMotor.getVelocity().getValueAsDouble(), 0.0, 0.01))) {
    //     stopScoringMotor();
    //     hasAlgae = true;
    //   }
    // }
  }
}
