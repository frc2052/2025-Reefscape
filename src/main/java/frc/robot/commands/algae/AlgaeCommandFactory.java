package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeCommandFactory {
  private static final AlgaeSubsystem algaeArm = AlgaeSubsystem.getInstance();

  public static Command intake() {
    return Commands.runEnd(() -> algaeArm.intake(), () -> algaeArm.stopScoringMotor(), algaeArm);
  }

  public static Command score() {
    return Commands.runEnd(() -> algaeArm.score(), () -> algaeArm.stopScoringMotor(), algaeArm);
  }
}
