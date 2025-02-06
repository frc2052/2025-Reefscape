package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class SuperstructureCommandFactory {

  private static Command levelOnePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L1),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.L1),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level One Position Command");
  }

  private static Command levelTwoPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L2),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.MID_LEVEL),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level Two Position Command");
  }

  private static Command levelThreePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L3),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.MID_LEVEL),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level Three Position Command");
  }

  private static Command levelFourPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.L4),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.L4),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level Four Position Command");
  }

  private static Command travelPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.TRAVEL),
                ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)))
        .withName("Travel Position Command");
  }

  private static Command descoreHighAlgaePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.UPPER_ALGAE),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.UPPER_ALGAE_DESCORE),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Descoring High Algae Position Command");
  }

  private static Command descoreLowerAlgaePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.LOWER_ALGAE),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.LOWER_ALGAE_DESCORE),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Descoring Lower Algae Position Command");
  }

  private static Command handoffPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL)
                .until(() -> ArmSubsystem.getInstance().isAtPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(ElevatorPosition.HANDOFF),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(ArmPosition.HANDOFF),
                    ArmCommandFactory.setArmPosition(ArmPosition.TRAVEL),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Handoff Position Command");
  }

  public enum ToLevel {
    L1(levelOnePositionCommand()),
    L2(levelTwoPositionCommand()),
    L3(levelThreePositionCommand()),
    L4(levelFourPositionCommand()),
    TRAVEL(travelPositionCommand()),
    DESCORE_HIGH_ALGAE(descoreHighAlgaePositionCommand()),
    DESCORE_LOW_ALGAE(descoreLowerAlgaePositionCommand()),
    HANDOFF(handoffPositionCommand());

    private final Command command;

    ToLevel(Command command) {
      this.command = command;
    }

    public Command getCommand() {
      return command;
    }
  }

  private static Command scoreLevelOneCommand() {
    return Commands.sequence(
        ToLevel.L1
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(ElevatorPosition.L1)
                        && ArmSubsystem.getInstance().isAtPosition(5, ArmPosition.L1.getAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  private static Command scoreLevelTwoCommand() {
    return Commands.sequence(
        ToLevel.L2
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(ElevatorPosition.L2)
                        && ArmSubsystem.getInstance()
                            .isAtPosition(5, ArmPosition.MID_LEVEL.getAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  private static Command scoreLevelThreeCommand() {
    return Commands.sequence(
        ToLevel.L3
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(ElevatorPosition.L3)
                        && ArmSubsystem.getInstance()
                            .isAtPosition(5, ArmPosition.MID_LEVEL.getAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  private static Command scoreLevelFourCommand() {
    return Commands.sequence(
        ToLevel.L4
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(ElevatorPosition.L4)
                        && ArmSubsystem.getInstance().isAtPosition(5, ArmPosition.L4.getAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  public enum ScoreLevel {
    L1(scoreLevelOneCommand()),
    L2(scoreLevelTwoCommand()),
    L3(scoreLevelThreeCommand()),
    L4(scoreLevelFourCommand());

    private final Command command;

    ScoreLevel(Command command) {
      this.command = command;
    }

    public Command getCommand() {
      return command;
    }
  }
}
