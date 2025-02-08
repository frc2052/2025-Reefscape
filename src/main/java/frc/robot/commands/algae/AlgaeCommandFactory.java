package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.AlgaePosition;

public class AlgaeCommandFactory {
  private static final AlgaeSubsystem algaeArm = AlgaeSubsystem.getInstance();

  public static Command setAlgaeArmPosition(AlgaePosition position) {
    return Commands.runOnce(() -> algaeArm.setGoalPosition(position));
  }

  public static Command intake() {
    return Commands.runOnce(() -> algaeArm.intake(), algaeArm)
        .finallyDo(() -> algaeArm.stopScoringMotor());
  }

  public static Command score() {
    return Commands.runOnce(() -> algaeArm.score(), algaeArm)
        .finallyDo(() -> algaeArm.stopScoringMotor());
  }
}
