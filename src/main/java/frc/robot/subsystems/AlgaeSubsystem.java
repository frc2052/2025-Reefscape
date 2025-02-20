// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.controlboard.PositionSuperstructure;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;
import frc.robot.util.Ports;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
  private static AlgaeSubsystem INSTANCE;

  private final TalonFX pivotMotor;
  private final TalonFX scoringMotor;

  private final AnalogEncoder pivotEncoder;

  private Angle goalPosition;

  private PIDController controller;

  private boolean isIntaking = false;
  private static boolean hasAlgae = false;
  private DelayedBoolean intakingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);

  public static AlgaeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AlgaeSubsystem();
    }
    return INSTANCE;
  }

  public AlgaeSubsystem() {
    pivotMotor = new TalonFX(Ports.ALGAE_PIVOT_ID);
    scoringMotor = new TalonFX(Ports.ALGAE_SCORING_ID);
    pivotEncoder = new AnalogEncoder(Ports.ALGAE_ENCODER_ID);

    pivotMotor.getConfigurator().apply(AlgaeArmConstants.PIVOT.MOTOR_CONFIG);

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.SupplyCurrentLimit = AlgaeArmConstants.SCORER.MOTOR_CURRENT_LIMIT;
    limitConfigs.SupplyCurrentLowerTime = (0.75);
    limitConfigs.SupplyCurrentLowerLimit = (5.0);
    limitConfigs.SupplyCurrentLimitEnable = false;

    scoringMotor
        .getConfigurator()
        .apply(AlgaeArmConstants.SCORER.MOTOR_CONFIG.withCurrentLimits(limitConfigs));

    controller =
        new PIDController(
            AlgaeArmConstants.PIVOT.P, AlgaeArmConstants.PIVOT.I, AlgaeArmConstants.PIVOT.D);

    goalPosition = PositionSuperstructure.getInstance().getTargetAction().getAlgaeArmPosition();
  }

  private void setPivotAngle(Angle angle) {
    if (angle == goalPosition && isAtPosition(AlgaeArmConstants.PIVOT.TOLERANCE, goalPosition)) {
      return;
    }

    if (hasAlgae && angle.in(Degrees) < 90) {
      return;
    }

    // pivotMotor.set(controller.calculate(pivotEncoder.get(), angle.in(Rotations)));
  }

  public void setGoalPosition(TargetAction position) {
    this.goalPosition = position.getAlgaeArmPosition();
    setPivotAngle(goalPosition);
  }

  public Command runPivotPct(double pct) {
    return Commands.runOnce(() -> setPivotSpeed(pct), this);
  }

  private void setPivotSpeed(double pct) {
    pivotMotor.set(pct);
    // pivotMotor.setControl(new TorqueCurrentFOC(pct * 40)); // percent of 40 amps
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

  public boolean isAtPosition(Angle goal) {
    return pivotMotor.getPosition().getValueAsDouble()
            < goal.in(Degrees) + AlgaeArmConstants.PIVOT.TOLERANCE
        && pivotMotor.getPosition().getValueAsDouble()
            > goal.in(Degrees) - AlgaeArmConstants.PIVOT.TOLERANCE;
  }

  public boolean isAtPosition(double tol, Angle goal) {
    return pivotMotor.getPosition().getValueAsDouble() < goal.in(Degrees) + tol
        && pivotMotor.getPosition().getValueAsDouble() > goal.in(Degrees) - tol;
  }

  public void stopScoringMotor() {
    scoringMotor.stopMotor();
    isIntaking = false;
  }

  public static boolean getHasAlgae(){
    return hasAlgae;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Algae Arm Angle", Units.radiansToDegrees(pivotEncoder.get()));
    Logger.recordOutput("Algae Arm Goal Angle", goalPosition.in(Degrees));
    Logger.recordOutput("Algae Arm At Goal", isAtPosition(goalPosition));

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
