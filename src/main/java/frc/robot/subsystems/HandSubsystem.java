// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class HandSubsystem extends SubsystemBase {
  private final TalonFX motor;
  private final CANrange range;
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
    range = new CANrange(Ports.HAND_CAN_RANGE);

    CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
    TalonFXConfiguration config = new TalonFXConfiguration();

    rangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    range.getConfigurator().apply(rangeConfig);

    InvertedValue inverted =
        Constants.HandConstants.HAND_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.withMotorOutput(new MotorOutputConfigs().withInverted(inverted));

    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.SupplyCurrentLimit = Constants.HandConstants.HAND_MOTOR_CURRENT_LIMIT;
    limitConfigs.SupplyCurrentLowerTime = (0.15);
    limitConfigs.SupplyCurrentLowerLimit = (1.0);
    limitConfigs.SupplyCurrentLimitEnable = false;

    motor
        .getConfigurator()
        .apply(
            config
                .withCurrentLimits(limitConfigs)
                .withAudio(new AudioConfigs().withBeepOnBoot(false)));
  }

  private void setMotor(double speed) {
    motor.set(speed);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public void motorOut() {
    setMotor(Constants.HandConstants.OUT_HAND_MOTOR_SPEED);
  }

  public void motorIn() {
    setMotor(-Constants.HandConstants.IN_HAND_MOTOR_SPEED);
  }

  public double motorVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  public boolean getHasCoral() {
    return range.getDistance().getValueAsDouble() < Constants.HandConstants.HAND_RANGE_THRESHOLD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Hand/Motor Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Hand/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Hand/Has Coral", getHasCoral());
    Logger.recordOutput("Hand/ToF Distance", range.getDistance().getValueAsDouble());
  }
}
