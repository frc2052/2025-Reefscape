package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeShooterSubsystem;

public class AlgaeCommandFactory {
    private static final AlgaeShooterSubsystem algaeShooter = AlgaeShooterSubsystem.getInstance();

    public static Command intake() {
        return Commands.runEnd(() -> algaeShooter.intake(), () -> algaeShooter.stopScoringMotor(), algaeShooter);
    }

    public static Command outtake() {
        return Commands.runEnd(() -> algaeShooter.outtake(), () -> algaeShooter.stopScoringMotor(), algaeShooter);
    }
}
