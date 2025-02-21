// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
  private static AlgaeSubsystem INSTANCE;

  private final TalonFX pivotMotor;
  private final TalonFX scoringMotor;

  private Angle goalPosition;

  private PIDController controller;

  private boolean isIntaking = false;
  private boolean hasAlgae = false;
  private DelayedBoolean intakingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);

  public static AlgaeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AlgaeSubsystem();
    }
    return INSTANCE;
  }

  private AlgaeSubsystem() {
    pivotMotor = new TalonFX(Ports.ALGAE_PIVOT_ID);
    scoringMotor = new TalonFX(Ports.ALGAE_SCORING_ID);

    pivotMotor.getConfigurator().apply(AlgaeArmConstants.PIVOT.MOTOR_CONFIG);

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.SupplyCurrentLimit = AlgaeArmConstants.SCORER.MOTOR_CURRENT_LIMIT;
    limitConfigs.SupplyCurrentLowerTime = (0.75);
    limitConfigs.SupplyCurrentLowerLimit = (5.0);
    limitConfigs.SupplyCurrentLimitEnable = false;

    scoringMotor
        .getConfigurator()
        .apply(AlgaeArmConstants.SCORER.MOTOR_CONFIG.withCurrentLimits(limitConfigs));

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

  public void intake() {
    scoringMotor.set(AlgaeArmConstants.SCORER.INTAKE_SPEED);
    isIntaking = true;
  }

  public void score() {
    scoringMotor.set(AlgaeArmConstants.SCORER.SCORE_SPEED);
    isIntaking = false;
    hasAlgae = false;
  }

  public Angle getPivotDesiredPosition() {
    return goalPosition;
  }

  public Angle getPivotAngle() {
    return pivotMotor.getPosition().getValue();
  }

  public boolean isAtDesiredPosition() {
    return isAtDesiredPosition(AlgaeArmConstants.PIVOT.TOLERANCE);
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

  public void stopScoringMotor() {
    scoringMotor.stopMotor();
    isIntaking = false;
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
