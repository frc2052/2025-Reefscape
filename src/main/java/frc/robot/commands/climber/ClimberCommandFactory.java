// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimberSubsystem;

/** Add your docs here. */
public class ClimberCommandFactory {
    private static final ClimberSubsystem climber = ClimberSubsystem.getInstance();

    public static Command climberUp() {
        return Commands.runEnd(() -> climber.moveUp(false), () -> climber.stop(), climber);
    }

    public static Command climberDown() {
        return Commands.runEnd(() -> climber.moveDown(false), () -> climber.stop(), climber);
    }

    public static Command climberFineUp() {
        return Commands.runEnd(() -> climber.moveUp(true), () -> climber.stop(), climber);
    }

    public static Command climberFineDown() {
        return Commands.runEnd(() -> climber.moveDown(true), () -> climber.stop(), climber);
    }

    public static Command setCoast() {
        return Commands.runOnce(() -> climber.setNeutralMode(NeutralModeValue.Coast), climber);
    }

    public static Command setBrake() {
        return Commands.runOnce(() -> climber.setNeutralMode(NeutralModeValue.Brake), climber);
    }
}
