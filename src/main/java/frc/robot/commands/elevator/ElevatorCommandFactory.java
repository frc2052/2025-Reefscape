// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommandFactory {
    private static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    public static Command manualUp() {
        return Commands.runEnd(() -> elevator.manualUp(), () -> elevator.stopElevator(), elevator);
    }

    public static Command manualDown() {
        return Commands.runEnd(() -> elevator.manualDown(), () -> elevator.stopElevator(), elevator);
    }

    public static Command setCoast() {
        return Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Coast), elevator);
    }

    public static Command setBrake() {
        return Commands.runOnce(() -> elevator.setNeutralMode(NeutralModeValue.Brake), elevator);
    }
}
