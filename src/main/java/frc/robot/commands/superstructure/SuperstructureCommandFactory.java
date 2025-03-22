package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;

public class SuperstructureCommandFactory {

    public static Command setCoast() {
        return Commands.sequence(
                IntakeCommandFactory.setCoast(),
                ElevatorCommandFactory.setCoast(),
                ArmCommandFactory.setCoast(),
                ClimberCommandFactory.setCoast());
    }

    public static Command setBrake() {
        return Commands.sequence(
                IntakeCommandFactory.setBrake(),
                ElevatorCommandFactory.setBrake(),
                ArmCommandFactory.setBrake(),
                ClimberCommandFactory.setBrake());
    }
}
