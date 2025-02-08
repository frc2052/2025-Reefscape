// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.Ports;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX pivotMotor;
  private ArmPosition goalPosition;
  private static ArmSubsystem INSTANCE;

  public static ArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ArmSubsystem();
    }
    return INSTANCE;
  }
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    goalPosition = ArmPosition.HANDOFF;

    pivotMotor = new TalonFX(Ports.ARM_TALONFX_ID);

    pivotMotor.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);
  }

  private void setArmAngle(Angle angle) {
    pivotMotor.setPosition(angle);
  }

  public void setArmPosition(ArmPosition position) {
    this.goalPosition = position;
  }

  public ArmPosition getArmPosition() {
    return goalPosition;
  }

  public Angle getArmAngle() {
    return pivotMotor.getPosition().getValue();
  }

  public boolean isAtPosition(double error, Angle goal) {
    return pivotMotor.getPosition().getValueAsDouble() < goal.in(Degrees) + ArmConstants.DEG_TOL
        && pivotMotor.getPosition().getValueAsDouble() > goal.in(Degrees) - ArmConstants.DEG_TOL;
  }

  @Override
  public void periodic() {
    setArmAngle(goalPosition.getAngle());
  }

  public enum ArmPosition { // TODO: set angles for each position
    HANDOFF(Degrees.of(0)),
    TRAVEL(Degrees.of(0)),
    L1(Degrees.of(0)),
    MID_LEVEL(Degrees.of(0)), // for both L2 and L3
    L4(Degrees.of(0)),
    UPPER_ALGAE_DESCORE(Degrees.of(0)),
    LOWER_ALGAE_DESCORE(Degrees.of(0));

    private final Angle angle;

    ArmPosition(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }
}
