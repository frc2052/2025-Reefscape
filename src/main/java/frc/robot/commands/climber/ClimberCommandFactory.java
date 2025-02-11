// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;

/** Add your docs here. */
public class ClimberCommandFactory {
  private static final ClimberSubsystem climber = ClimberSubsystem.getInstance();

  public static Command climberUp() {
    return Commands.runOnce(() -> climber.moveUp(false), climber);
  }

  public static Command climberDown() {
    return Commands.runOnce(() -> climber.moveDown(false), climber);
  }

  public static Command climberFineUp() {
    return Commands.runOnce(() -> climber.moveUp(true), climber);
  }

  public static Command climberFineDown() {
    return Commands.runOnce(() -> climber.moveDown(true), climber);
  }
}
