// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

/** Add your docs here. */
public class ElevatorCommandFactory {
  private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

  public static Command setElevatorPosition(ElevatorPosition position) {
    return Commands.runOnce(() -> elevator.setElevatorPosition(position), elevator);
  }

  public static Command manualUp() {
    return Commands.runEnd(() -> elevator.manualUp(), () -> elevator.stopElevator(), elevator);
  }

  public static Command manualDown() {
    return Commands.runEnd(() -> elevator.manualDown(), () -> elevator.stopElevator(), elevator);
  }
}
