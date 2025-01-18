// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbingSubsystem;

/** Add your docs here. */
public class ClimberCommandFactory {
    private static final ClimbingSubsystem climber = ClimbingSubsystem.getInstance();

    public static Command climberUp() {
        return Commands.runEnd(()-> climber.moveUpPositionSpeed(Constants.ClimberConstants.climberSpeed), ()-> climber.stopMotors(),climber);
    }
    public static Command climberDown() {
        return Commands.runEnd(()-> climber.moveDownPositionSpeed(Constants.ClimberConstants.climberSpeed), ()-> climber.stopMotors(),climber);
    }
}
