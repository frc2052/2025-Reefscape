package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class IntakeRollerCommandFactory {
    private static final IntakeRollerSubsystem rollers = IntakeRollerSubsystem.getInstance();

    public static Command intake() {
        return Commands.runEnd(() -> rollers.intake(), () -> rollers.stopMotor(), rollers);
    }

    public static Command outtake() {
        return Commands.runEnd(() -> rollers.outtake(), () -> rollers.stopMotor(), rollers);
    }
}
