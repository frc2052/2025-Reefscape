package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class SuperstructureCommandFactory {

  private static Command levelOnePositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L1),
        new ConditionalCommand(
            ArmCommandFactory.setArmPosition(ArmPosition.L1),
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
            () -> ElevatorSubsystem.getInstance().atPosition()));
  }

  private static Command levelTwoPositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L2),
        new ConditionalCommand(
            ArmCommandFactory.setArmPosition(ArmPosition.MID_LEVEL),
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
            () -> ElevatorSubsystem.getInstance().atPosition()));
  }

  private static Command levelThreePositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L3),
        new ConditionalCommand(
            ArmCommandFactory.setArmPosition(ArmPosition.MID_LEVEL),
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
            () -> ElevatorSubsystem.getInstance().atPosition()));
  }

  private static Command levelFourPositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L4),
        new ConditionalCommand(
            ArmCommandFactory.setArmPosition(ArmPosition.L4),
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
            () -> ElevatorSubsystem.getInstance().atPosition()));
  }

  private static Command travelPositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.TRAVEL),
        ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL));
  }

  private static Command descoreHighAlgaePositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.UPPER_ALGAE),
        ArmCommandFactory.setArmPosition(ArmPosition.UPPER_ALGAE_DESCORE));
  }

  private static Command descoreLowerAlgaePositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.LOWER_ALGAE),
        ArmCommandFactory.setArmPosition(ArmPosition.LOWER_ALGAE_DESCORE));
  }

  private static Command handoffPositionCommand() {
    return Commands.parallel(
        ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.HANDOFF),
        ArmCommandFactory.setArmPosition(ArmPosition.HANDOFF));
  }

  public enum Level {
    L1(levelOnePositionCommand()),
    L2(levelTwoPositionCommand()),
    L3(levelThreePositionCommand()),
    L4(levelFourPositionCommand()),
    TRAVEL(travelPositionCommand()),
    DESCORE_HIGH_ALGAE(descoreHighAlgaePositionCommand()),
    DESCORE_LOW_ALGAE(descoreLowerAlgaePositionCommand()),
    HANDOFF(handoffPositionCommand());

    private final Command command;

    Level(Command command) {
      this.command = command;
    }

    public Command getCommand() {
      return command;
    }
  }
}
