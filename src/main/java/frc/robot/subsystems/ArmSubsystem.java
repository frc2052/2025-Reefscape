// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotState;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;
import frc.robot.util.Ports;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final RobotState robotState = RobotState.getInstance();
  private final TalonFX pivotMotor;
  private Angle goalPosition;

  private static ArmSubsystem INSTANCE;

  public static ArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ArmSubsystem();
    }
    return INSTANCE;
  }

  private ArmSubsystem() {
    goalPosition = TargetAction.HP.coralArmAngle;

    pivotMotor = new TalonFX(Ports.ARM_TALONFX_ID);

    pivotMotor.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);
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
      System.out.println("Arm at goal position");
      return;
    }

    pivotMotor.setControl(new PositionVoltage(angle));
  }

  public void setArmPosition(TargetAction position) {
    this.goalPosition = clampPosition(position.getCoralArmAngle());
    setPivotAngle(goalPosition);
  }

  private Angle clampPosition(Angle pos) {
    if (robotState.getHasCoral()) {
      if (pos.in(Degrees) < ArmConstants.MIN_CORAL_ANGLE.in(Degrees)) {
        System.out.println("DESIRED ANGLE BEYOND MIN LIMIT");
        return ArmConstants.MIN_CORAL_ANGLE;
      } else if (pos.in(Degrees) > ArmConstants.MAX_CORAL_ANGLE.in(Degrees)) {
        System.out.println("DESIRED ANGLE BEYOND MAX LIMIT");
        return ArmConstants.MAX_CORAL_ANGLE;
      }
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
    return isAtDesiredPosition(ArmConstants.DEG_TOL);
  }

  public boolean isAtDesiredPosition(double tol) {
    return isAtPosition(tol, goalPosition);
  }

  public boolean isAtPosition(double tol, Angle goal) {
    return pivotMotor.getPosition().getValueAsDouble() < goal.in(Degrees) + tol
        && pivotMotor.getPosition().getValueAsDouble() > goal.in(Degrees) - tol;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Arm Angle", Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()));
    Logger.recordOutput("Arm Goal Angle", goalPosition.in(Degrees));
    Logger.recordOutput("Arm Motor Set Speed", pivotMotor.get());
    Logger.recordOutput("Arm Velocity", pivotMotor.getVelocity().getValueAsDouble());
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
