// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;

/** Add your docs here. */
public class ElevatorCommandFactory {

    public static Command setElevatorPosition(ElevatorPosition position) {
        return new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPosition(position));
    }

    public static Command manualUp() {
        return new Command() {
            @Override
            public void execute() {
                ElevatorSubsystem.getInstance().manualUp();
            }
        };
    }

    public static Command manualDown() {
        return new Command() {
            @Override
            public void execute() {
                ElevatorSubsystem.getInstance().manualDown();
            }
        };
    }
}
