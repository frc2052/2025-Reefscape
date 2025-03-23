package frc.robot.commands.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class SuperstructureCommandFactory {

    public static Command setCoast() {
        return Commands.sequence(
                IntakeCommandFactory.setCoast(),
                ElevatorCommandFactory.setCoast(),
                ArmCommandFactory.setCoast(),
                ClimberCommandFactory.setCoast(),
                new InstantCommand(() -> DrivetrainSubsystem.getInstance().setNeutralMode(NeutralModeValue.Coast)));
    }

    public static Command setBrake() {
        return Commands.sequence(
                IntakeCommandFactory.setBrake(),
                ElevatorCommandFactory.setBrake(),
                ArmCommandFactory.setBrake(),
                ClimberCommandFactory.setBrake(),
                new InstantCommand(() -> DrivetrainSubsystem.getInstance().setNeutralMode(NeutralModeValue.Brake)));
    }
}
