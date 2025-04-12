package frc.robot.commands.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class IntakeCommandFactory {
    private static final IntakeRollerSubsystem rollers = IntakeRollerSubsystem.getInstance();
    private static final IntakePivotSubsystem pivot = IntakePivotSubsystem.getInstance();

    public static Command intake() {
        return Commands.runEnd(() -> rollers.intake(), () -> rollers.stopMotor(), rollers);
    }

    public static Command intakeAlgae() {
        return Commands.runEnd(() -> rollers.intakeAlgae(), () -> rollers.stopMotor(), rollers);
    }

    public static Command outtake() {
        return Commands.runEnd(() -> rollers.outtake(), () -> rollers.stopMotor(), rollers);
    }

    public static Command setCoast() {
        return new InstantCommand(() -> pivot.setNeutralMode(NeutralModeValue.Coast), pivot);
    }

    public static Command setBrake() {
        return new InstantCommand(() -> pivot.setNeutralMode(NeutralModeValue.Brake), pivot);
    }
}
