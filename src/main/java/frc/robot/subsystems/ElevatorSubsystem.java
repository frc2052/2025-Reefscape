// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Ports;

public class ElevatorSubsystem extends SubsystemBase {
  private static TalonFX leftMotor;
  private static TalonFX rightMotor;

  private double goalPositionTicks;
  private double previousPositionTicks;

  private MotionMagicTorqueCurrentFOC elevatorRequest;

  private static ElevatorSubsystem INSTANCE;

  public static ElevatorSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ElevatorSubsystem();
    }
    return INSTANCE;
  }

  private ElevatorSubsystem() {
    goalPositionTicks = ElevatorPosition.STOW.getPositionTicks();
    elevatorRequest = new MotionMagicTorqueCurrentFOC(goalPositionTicks);

    leftMotor = new TalonFX(Ports.ELEVATOR_LEFT_ID);
    rightMotor = new TalonFX(Ports.ELEVATOR_RIGHT_ID);

    rightMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
    leftMotor
        .getConfigurator()
        .apply(
            ElevatorConstants.MOTOR_CONFIG
                .HardwareLimitSwitch
                .withReverseLimitEnable(true)
                .withReverseLimitRemoteSensorID(Ports.ELEVATOR_LIMIT_SWITCH));

    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
  }

  public void setPosition(ElevatorPosition elevatorPosition) {
    setPositionTicks(elevatorPosition.getPositionTicks());
  }

  public void setPositionTicks(double elevatorPositionTicks) {
    if (goalPositionTicks == elevatorPositionTicks && atPosition()) {
      return;
    }

    previousPositionTicks = goalPositionTicks;
    goalPositionTicks = elevatorPositionTicks;

    // set target position to 100 rotations
    leftMotor.setControl(elevatorRequest.withPosition(goalPositionTicks));
  }

  public Command manualUp() {
    return Commands.runOnce(
        () -> {
          leftMotor.set(ElevatorConstants.MANUAL_MOTOR_SPEED);
        });
  }

  public Command manualDown() {
    return Commands.runOnce(
        () -> {
          if (!elevatorZeroed()) {
            leftMotor.set(-ElevatorConstants.MANUAL_MOTOR_SPEED);
          }
        });
  }

  public boolean elevatorZeroed() {
    return leftMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public Command stopElevator() {
    return Commands.runOnce(() -> leftMotor.set(0.0));
  }

  public boolean atPosition() {
    return Math.abs(goalPositionTicks - leftMotor.getPosition().getValueAsDouble())
        <= ElevatorConstants.TICKS_DEADZONE;
  }

  public void zeroEncoder() {
    leftMotor.getConfigurator().setPosition(0);
  }

  @Override
  public void periodic() {

    if (elevatorZeroed() || leftMotor.getPosition().getValueAsDouble() <= 0) {
      zeroEncoder();

      // If the elevator is traveling downwards stop the belt motor and end the current command.
      if (goalPositionTicks < previousPositionTicks) {
        goalPositionTicks = 0;
        stopElevator().schedule();
      }
    } else {
      if (atPosition()) {
        stopElevator().schedule();
      }
    }
  }

  public static enum ElevatorPosition {
    STOW(0),
    HANDOFF(0),
    L1(0),
    L2(0),
    L3(0),
    L4(0),
    LOWER_ALGAE(0),
    UPPER_ALGAE(0),
    TRAVEL(0);

    private final int positionTicks;

    private ElevatorPosition(int positionTicks) {
      this.positionTicks = positionTicks;
    }

    public int getPositionTicks() {
      return positionTicks;
    }
  }
}
