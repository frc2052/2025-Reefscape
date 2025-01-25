// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Ports;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private static TalonFX frontMotor;
  private static TalonFX backMotor;

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

    frontMotor = new TalonFX(Ports.ELEVATOR_FRONT_ID, "Krawlivore");
    backMotor = new TalonFX(Ports.ELEVATOR_BACK_ID, "Krawlivore");

    backMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
    frontMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
    // .HardwareLimitSwitch
    // .withReverseLimitEnable(true)
    // .withReverseLimitRemoteSensorID(Ports.ELEVATOR_LIMIT_SWITCH));

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), true));
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
    frontMotor.setControl(elevatorRequest.withPosition(goalPositionTicks));
  }

  public void manualManualUp() {
    frontMotor.set(ElevatorConstants.MANUAL_MOTOR_SPEED);
  }

  public void manualManualDown() {
    frontMotor.set(-ElevatorConstants.MANUAL_MOTOR_SPEED);
  }

  public Command manualUp() {
    return new InstantCommand(
        () -> {
          frontMotor.set(ElevatorConstants.MANUAL_MOTOR_SPEED);
        },
        this);
  }

  public Command manualDown() {
    return new InstantCommand(
        () -> {
          // if (getPosition() > 0) {
          frontMotor.set(-ElevatorConstants.MANUAL_MOTOR_SPEED);
          // }
        },
        this);
  }

  public double getPosition() {
    return frontMotor.getPosition().getValueAsDouble();
  }

  public void homeElevator() {
    frontMotor.getConfigurator().apply(ElevatorConstants.HOMING_CURRENT_LIMIT_CONFIG);
    // manualDown().(() -> (frontMotor.getStatorCurrent().getValueAsDouble() > 2.0));
  }

  public Command stopElevator() {
    return new InstantCommand(() -> frontMotor.set(0.0));
  }

  public boolean atPosition() {
    return Math.abs(goalPositionTicks - frontMotor.getPosition().getValueAsDouble())
        <= ElevatorConstants.TICKS_DEADZONE;
  }

  public boolean atPosition(ElevatorPosition position) {
    return Math.abs(position.positionTicks - frontMotor.getPosition().getValueAsDouble())
        <= ElevatorConstants.TICKS_DEADZONE;
  }

  public void zeroEncoder() {
    frontMotor.getConfigurator().setPosition(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator Position", getPosition());
    Logger.recordOutput("Elevator Goal Position", goalPositionTicks);
    Logger.recordOutput("Elevator At Goal Position", atPosition());

    // if (elevatorZeroed() || leftMotor.getPosition().getValueAsDouble() <= 0) {
    //   zeroEncoder();

    //   // If the elevator is traveling downwards stop the belt motor and end the current command.
    //   if (goalPositionTicks < previousPositionTicks) {
    //     goalPositionTicks = 0;
    //     stopElevator().schedule();
    //   }
    // } else {
    //   if (atPosition()) {
    //     stopElevator().schedule();
    //   }
    // }
  }

  public static enum ElevatorPosition {
    STOW(5),
    HANDOFF(5),
    L1(5),
    L2(5),
    L3(5),
    L4(5),
    LOWER_ALGAE(5),
    UPPER_ALGAE(5),
    TRAVEL(2.25);

    private final double positionTicks;

    private ElevatorPosition(double positionTicks) {
      this.positionTicks = positionTicks;
    }

    public double getPositionTicks() {
      return positionTicks;
    }
  }
}
