// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

import org.littletonrobotics.junction.Logger;

public class CoralArmSubsystem extends SubsystemBase {
  private final TalonFX pivotMotor;
  private Angle goalPosition;

  private static CoralArmSubsystem INSTANCE;

  public static CoralArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new CoralArmSubsystem();
    }
    return INSTANCE;
  }

  private CoralArmSubsystem() {
    goalPosition =
        SuperstructureSubsystem.getInstance().getSelectedTargetAction().getCoralArmAngle();

    pivotMotor = new TalonFX(Ports.ARM_TALONFX_ID);

    pivotMotor.getConfigurator().apply(CoralArmConstants.MOTOR_CONFIG);
  }

  public Command runPct(double pct) {
    return Commands.runOnce(() -> setPivotSpeed(pct), this);
  }

  private void setPivotSpeed(double pct) {
    pivotMotor.set(pct);
  }

  private void setPivotVolts(Voltage v) {
    pivotMotor.setVoltage(v.in(Volts));
  }

  private void setPivotAngle(Angle angle) {
    if (angle == goalPosition && isAtDesiredPosition()) {
      return;
    }

    pivotMotor.setControl(new PositionVoltage(angle));
  }

  public void setArmPosition(TargetAction position) {
    this.goalPosition = clampPosition(position.getCoralArmAngle());
    setPivotAngle(goalPosition);
  }

  private Angle clampPosition(Angle pos) {
    if (pos.in(Degrees) < CoralArmConstants.MIN_CORAL_ARM_ANGLE.in(Degrees)) {
      System.out.println("DESIRED ANGLE BEYOND MIN LIMIT");
      return CoralArmConstants.MIN_CORAL_ARM_ANGLE;
    } else if (pos.in(Degrees) > CoralArmConstants.MAX_CORAL_ARM_ANGLE.in(Degrees)) {
      System.out.println("DESIRED ANGLE BEYOND MAX LIMIT");
      return CoralArmConstants.MAX_CORAL_ARM_ANGLE;
    }

    return pos;
  }

  public Angle getArmDesiredPosition() {
    return goalPosition;
  }

  public Angle getArmAngle() {
    return pivotMotor.getPosition().getValue();
  }

  public boolean isAtDesiredPosition() {
    return isAtDesiredPosition(CoralArmConstants.DEG_TOL);
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
    Logger.recordOutput("Coral Arm/Angle", getPosition().in(Degrees));
    Logger.recordOutput("Coral Arm/Goal Angle", goalPosition.in(Degrees));
    Logger.recordOutput("Coral Arm/Motor Set Speed", pivotMotor.get());
    Logger.recordOutput("Coral Arm/Velocity", pivotMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Coral Arm/At Goal", isAtDesiredPosition(5));
  }

  /* SysId routine for characterizing arm. This is used to find PID gains for the arm motor. */
  private final SysIdRoutine m_sysIdRoutineArm =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdArm_State", state.toString())),
          new SysIdRoutine.Mechanism(this::setPivotVolts, null, this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineArm;
  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }
}
