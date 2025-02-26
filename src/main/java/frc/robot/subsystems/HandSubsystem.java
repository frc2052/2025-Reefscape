// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HandConstants;
import frc.robot.RobotState;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class HandSubsystem extends SubsystemBase {
  private final TalonFX motor;
  private final CANrange range;
  private static HandSubsystem INSTANCE;
  private boolean isIntaking = false;

  public static HandSubsystem getInstance() {
    if (INSTANCE == null) {
      return new HandSubsystem();
    }
    return INSTANCE;
  }

  public HandSubsystem() {
    motor = new TalonFX(Ports.HAND_TALONFX_ID);
    range = new CANrange(Ports.HAND_CAN_RANGE);

    range.getConfigurator().apply(HandConstants.CANRANGE_CONFIG);

    motor.getConfigurator().apply(HandConstants.MOTOR_CONFIG);
  }

  private void setMotor(double speed) {
    motor.set(speed);
  }

  public void stopMotor() {
    isIntaking = false;
    motor.stopMotor();
  }

  public void motorOut() {
    setMotor(Constants.HandConstants.OUT_HAND_MOTOR_SPEED);
  }

  public void motorIn() {
    isIntaking = true;
    setMotor(-Constants.HandConstants.IN_HAND_MOTOR_SPEED);
  }

  public double motorVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  public boolean getHasCoral() {
    return range.getIsDetected().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Hand/Motor Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Hand/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Hand/Has Coral", getHasCoral());
    Logger.recordOutput("Hand/ToF Distance", range.getDistance().getValueAsDouble());
    RobotState.getInstance().setHasCoral(getHasCoral());
    RobotState.getInstance().setIsIntaking(isIntaking);
  }
}
