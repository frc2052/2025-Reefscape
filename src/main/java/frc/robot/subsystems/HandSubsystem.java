// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Ports;

public class HandSubsystem extends SubsystemBase {
  private final TalonFX motor;
  private static HandSubsystem INSTANCE;

  public static HandSubsystem getInstance() {
    if (INSTANCE == null) {
      return new HandSubsystem();
    }
    return INSTANCE;
  }
  /** Creates a new HandSubsystem. */
  public HandSubsystem() {
    motor = new TalonFX(Ports.HAND_TALONFX_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    InvertedValue inverted =
        Constants.HandConstants.HAND_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.withMotorOutput(new MotorOutputConfigs().withInverted(inverted));

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = Constants.HandConstants.HAND_MOTOR_CURRENT_LIMIT;
    limitConfigs.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(limitConfigs);
  }

  private void setMotor(double speed) {
    motor.set(speed);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public void motorOut() {
    setMotor(Constants.HandConstants.HAND_MOTOR_SPEED);
  }

  public void motorIn() {
    setMotor(-Constants.HandConstants.HAND_MOTOR_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
