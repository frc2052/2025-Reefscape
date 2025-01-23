package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.drive.AlignWithSpecificTagCommand;
import frc.robot.commands.drive.AlignWithTagCommand.AlignLocation;
import frc.robot.commands.superstructure.SuperstructureCommandFactory.ScoreLevel;

public class ReefScoringCommandFactory {

  private static Command ScoreLocation(ScoreLevel level, AlignLocation alignLocation, int tagID) {
    return Commands.sequence(
        new AlignWithSpecificTagCommand(alignLocation, () -> 0, () -> 0, () -> 0, tagID),
        level.getCommand());
  }
  // spotless:off
  public enum ReefScoringPosition {
    AB_TROUGH(ScoreLocation(ScoreLevel.L1,AlignLocation.MIDDLE,RobotState.getInstance().isRedAlliance() ? 7 : 18)),
    A_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 7 : 18)),
    B_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 7 : 18)),
    A_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 7 : 18)),
    B_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 7 : 18)),
    A_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 7 : 18)),
    B_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 7 : 18)),

    CD_TROUGH(ScoreLocation(ScoreLevel.L1,AlignLocation.MIDDLE,RobotState.getInstance().isRedAlliance() ? 8 : 17)),
    C_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 8 : 17)),
    D_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 8 : 17)),
    C_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 8 : 17)),
    D_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 8 : 17)),
    C_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 8 : 17)),
    D_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 8 : 17)),

    EF_TROUGH(ScoreLocation(ScoreLevel.L1,AlignLocation.MIDDLE,RobotState.getInstance().isRedAlliance() ? 11 : 22)),
    E_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 11 : 22)),
    F_L2(ScoreLocation(ScoreLevel.L2,AlignLocation.RIGHT,RobotState.getInstance().isRedAlliance() ? 11 : 22)),
    E_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 11 : 22)),
    F_L3(ScoreLocation(ScoreLevel.L3,AlignLocation.RIGHT,RobotState.getInstance().isRedAlliance() ? 11 : 22)),
    E_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 11 : 22)),
    F_L4(ScoreLocation(ScoreLevel.L4,AlignLocation.RIGHT,RobotState.getInstance().isRedAlliance() ? 11 : 22)),

    GH_TROUGH(ScoreLocation(ScoreLevel.L1,AlignLocation.MIDDLE,RobotState.getInstance().isRedAlliance() ? 10 : 21)),
    G_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 10 : 21)),
    H_L2(ScoreLocation(ScoreLevel.L2,AlignLocation.RIGHT,RobotState.getInstance().isRedAlliance() ? 10 : 21)),
    G_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 10 : 21)),
    H_L3(ScoreLocation(ScoreLevel.L3,AlignLocation.RIGHT,RobotState.getInstance().isRedAlliance() ? 10 : 21)),
    G_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 10 : 21)),
    H_L4(ScoreLocation(ScoreLevel.L4,AlignLocation.RIGHT,RobotState.getInstance().isRedAlliance() ? 10 : 21)),

    IJ_TROUGH(ScoreLocation(ScoreLevel.L1,AlignLocation.MIDDLE,RobotState.getInstance().isRedAlliance() ? 9 : 20)),
    I_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 9 : 20)),
    J_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 9 : 20)),
    I_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 9 : 20)),
    J_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 9 : 20)),
    I_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 9 : 20)),
    J_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 9 : 20)),

    KL_TROUGH(ScoreLocation(ScoreLevel.L1,AlignLocation.MIDDLE,RobotState.getInstance().isRedAlliance() ? 6 : 19)),
    K_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 6 : 19)),
    L_L2(ScoreLocation(ScoreLevel.L2, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 6 : 19)),
    K_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 6 : 19)),
    L_L3(ScoreLocation(ScoreLevel.L3, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 6 : 19)),
    K_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.LEFT, RobotState.getInstance().isRedAlliance() ? 6 : 19)),
    L_L4(ScoreLocation(ScoreLevel.L4, AlignLocation.RIGHT, RobotState.getInstance().isRedAlliance() ? 6 : 19));

    private final Command command;

    ReefScoringPosition(Command command) {
      this.command = command;
    }

    public Command getCommand() {
      return command;
    }
  }
  // spotless:on
}
