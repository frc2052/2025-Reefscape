// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem INSTANCE;
  private static ElevatorPosition currentPosition;

  private DigitalInput limitSwitch;
  private AnalogEncoder encoder;

  private PIDController contoller;

  private static TalonFX leftMotor;
  private static TalonFX rightMotor;

  private boolean isManual;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    currentPosition = ElevatorPosition.STOW;

    limitSwitch = new DigitalInput(Constants.ElevatorConstants.IDs.LIMIT_SWITCH);
    encoder = new AnalogEncoder(Constants.ElevatorConstants.IDs.ENCODER);

    leftMotor = new TalonFX(Constants.ElevatorConstants.IDs.LEFT_MOTOR);
    rightMotor = new TalonFX(Constants.ElevatorConstants.IDs.RIGHT_MOTOR);

    InvertedValue leftInverted =
        Constants.ElevatorConstants.LEFT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(leftInverted));

    InvertedValue rightInverted =
        Constants.ElevatorConstants.RIGHT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(rightInverted));

    contoller =
        new PIDController(
            Constants.ElevatorConstants.MotorPIDConstants.KP,
            Constants.ElevatorConstants.MotorPIDConstants.KI,
            Constants.ElevatorConstants.MotorPIDConstants.KD);
    contoller.setTolerance(Constants.ElevatorConstants.MotorPIDConstants.ERROR);

    leftMotor.getConfigurator().apply(leftMotorConfig);
    rightMotor.getConfigurator().apply(rightMotorConfig);

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    isManual = false;
  }

  public static ElevatorSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ElevatorSubsystem();
    }
    return INSTANCE;
  }

  public static ElevatorPosition getPosition() {
    return currentPosition;
  }

  public void setElevatorPosition(ElevatorPosition position) {
    currentPosition = position;
  }

  private void setPosition(Angle value) {
    if (limitSwitch.isAnalogTrigger()) {
      System.out.println("Elevator Limit Switch Triggered");
      return;
    }
    leftMotor.setPosition(contoller.calculate(encoder.get(), value.in(Degrees)));
    rightMotor.setPosition(contoller.calculate(encoder.get(), value.in(Degrees)));
  }

  private void setPosition(double elevatorPosition) {
    if (limitSwitch.isAnalogTrigger()) {
      System.out.println("Elevator Limit Switch Triggered");
      return;
    }
    leftMotor.setPosition(contoller.calculate(encoder.get(), elevatorPosition));
    rightMotor.setPosition(contoller.calculate(encoder.get(), elevatorPosition));
  }

  public void turnOnManualMode() {
    isManual = true;
  }

  public void turnOffManualMode() {
    isManual = false;
  }

  public void toggleManualMode() {
    isManual = !isManual;
  }

  public boolean getIsManual() {
    return isManual;
  }

  public Command manualUp() {
    if (!isManual) {
      return new Command() {};
    }

    return Commands.runOnce(
        () -> {
          leftMotor.set(Constants.ElevatorConstants.MANUL_MOTOR_SPEED);
          rightMotor.set(Constants.ElevatorConstants.MANUL_MOTOR_SPEED);
        });
  }

  public Command manualDown() {
    if (!isManual) {
      return new Command() {};
    }

    return Commands.runOnce(
        () -> {
          leftMotor.set(-Constants.ElevatorConstants.MANUL_MOTOR_SPEED);
          rightMotor.set(-Constants.ElevatorConstants.MANUL_MOTOR_SPEED);
        });
  }

  public Command stopElevator() {
    return Commands.runOnce(
        () -> {
          leftMotor.stopMotor();
          rightMotor.stopMotor();
        });
  }

  public boolean atPosition() {
    return contoller.atSetpoint();
  }

  public boolean atPosition(double setPoint) {
    contoller.setSetpoint(setPoint);
    Boolean isAtPoint = contoller.atSetpoint();
    setPosition(heightToAngle(currentPosition.getHeight()));
    return isAtPoint;
  }

  private Angle heightToAngle(Distance height) {
    return Degrees.of(height.in(Inches) * Constants.ElevatorConstants.DEGREES_TO_INCHES_RATIO);
  }

  @Override
  public void periodic() {
    if (isManual) {
      return;
    }
    setPosition(heightToAngle(currentPosition.getHeight()));
  }

  public enum ElevatorPosition {
    STOW(Inches.of(0)),
    HANDOFF(Inches.of(0)),
    L1(Inches.of(18)),
    L2(Inches.of(32)),
    L3(Inches.of(48)),
    L4(Inches.of(72)),
    LOWER_ALGAE(Inches.of(40)),
    UPPER_ALGAE(Inches.of(54)),
    TRAVEL(Inches.of(18));

    private final Distance positionHeight;

    private ElevatorPosition(Distance positionHeight) {
      this.positionHeight = positionHeight;
    }

    public Distance getHeight() {
      return positionHeight;
    }
  }
}
