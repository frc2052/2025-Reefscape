// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Ports;

public class ArmSubsystem extends SubsystemBase {
  private final CANcoder encoder;
  private final TalonFX motor;
  private final PIDController controller;
  private ArmPosition position;
  private static ArmSubsystem INSTANCE;

  public static ArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ArmSubsystem();
    }
    return INSTANCE;
  }
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    position = ArmPosition.HANDOFF;

    encoder = new CANcoder(Ports.ARM_CANCODER_ID);
    motor = new TalonFX(Ports.ARM_TALONFX_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    InvertedValue inverted =
        Constants.ArmConstants.ARM_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.withMotorOutput(new MotorOutputConfigs().withInverted(inverted));

    motor.getConfigurator().apply(config);

    controller =
        new PIDController(
            Constants.ArmConstants.PIDs.KP,
            Constants.ArmConstants.PIDs.KI,
            Constants.ArmConstants.PIDs.KD);
  }

  private void setArmAngle(Angle angle) {
    motor.set(controller.calculate(motor.getPosition().getValue().in(Degrees), angle.in(Degrees)));
  }

  public void setArmPosition(ArmPosition position) {
    this.position = position;
  }

  public ArmPosition getArmPosition() {
    return position;
  }

  public Angle getArmAngle() {
    return motor.getPosition().getValue();
  }

  public boolean isAtPosition(double error) {
    controller.setTolerance(error);
    return controller.atSetpoint();
  }

  public boolean isAtPosition(double error, Angle setPoint) {
    controller.setSetpoint(setPoint.in(Degrees));
    controller.setTolerance(error);
    Boolean isAtPoint = controller.atSetpoint();
    setArmAngle(position.getAngle());
    return isAtPoint;
  }

  @Override
  public void periodic() {
    setArmAngle(position.getAngle());
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
