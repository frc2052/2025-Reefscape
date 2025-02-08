package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SuperstructureCommandFactory {

  private static Command levelOnePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.L1),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.L1),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level One Position Command");
  }

  private static Command levelTwoPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.L2),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.UA),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level Two Position Command");
  }

  private static Command levelThreePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.L3),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.UA),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level Three Position Command");
  }

  private static Command levelFourPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.L4),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.L4),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Level Four Position Command");
  }

  private static Command travelPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.TR),
                ArmCommandFactory.setArmPosition(TargetAction.TR)))
        .withName("Travel Position Command");
  }

  private static Command descoreHighAlgaePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.UA),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.UA),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Descoring High Algae Position Command");
  }

  private static Command descoreLowerAlgaePositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.LA),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.LA),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
                    () -> ElevatorSubsystem.getInstance().atPosition())))
        .withName("Descoring Lower Algae Position Command");
  }

  private static Command handoffPositionCommand() {
    return Commands.sequence(
            ArmCommandFactory.setArmPosition(TargetAction.TR)
                .until(() -> ArmSubsystem.getInstance().isAtDesiredPosition(5)),
            Commands.parallel(
                ElevatorCommandFactory.setElevatorPosition(TargetAction.HP),
                new ConditionalCommand(
                    ArmCommandFactory.setArmPosition(TargetAction.HP),
                    ArmCommandFactory.setArmPosition(TargetAction.TR),
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
                    (ElevatorSubsystem.getInstance().atPosition(TargetAction.L1)
                        && ArmSubsystem.getInstance()
                            .isAtPosition(ArmConstants.DEG_TOL, TargetAction.L1.getCoralArmAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  private static Command scoreLevelTwoCommand() {
    return Commands.sequence(
        ToLevel.L2
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(TargetAction.L2)
                        && ArmSubsystem.getInstance()
                            .isAtPosition(ArmConstants.DEG_TOL, TargetAction.L2.getCoralArmAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  private static Command scoreLevelThreeCommand() {
    return Commands.sequence(
        ToLevel.L3
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(TargetAction.L3)
                        && ArmSubsystem.getInstance()
                            .isAtPosition(ArmConstants.DEG_TOL, TargetAction.L3.getCoralArmAngle()))),
        HandCommandFactory.motorOut().withTimeout(0.5));
  }

  private static Command scoreLevelFourCommand() {
    return Commands.sequence(
        ToLevel.L4
            .getCommand()
            .until(
                () ->
                    (ElevatorSubsystem.getInstance().atPosition(TargetAction.L4)
                        && ArmSubsystem.getInstance()
                            .isAtPosition(ArmConstants.DEG_TOL, TargetAction.L4.getCoralArmAngle()))),
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
