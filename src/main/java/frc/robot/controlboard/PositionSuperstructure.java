package frc.robot.controlboard;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;

public class PositionSuperstructure {
  private static PositionSuperstructure INSTANCE;
  private RobotState state = RobotState.getInstance();
  private ControlBoard controlBoard = ControlBoard.getInstance();

  private TargetFieldLocation targetReefSide = TargetFieldLocation.AB;
  private TargetAction targetAction = TargetAction.HP;
  private ReefSubSide reefSubSide = ReefSubSide.CENTER;

  public static PositionSuperstructure getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new PositionSuperstructure();
    }
    return INSTANCE;
  }

  private PositionSuperstructure() {
    controlBoard.setGoalL1L().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L1L)));
    controlBoard.setGoalL1H().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L1H)));
    controlBoard.setGoalL2().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L2)));
    controlBoard.setGoalL3().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L3)));
    controlBoard.setGoalL4().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L4)));
    controlBoard
        .setGoalLowerAlgae()
        .onTrue(new InstantCommand(() -> setTargetAction(TargetAction.LA)));
    controlBoard
        .setGoalUpperAlgae()
        .onTrue(new InstantCommand(() -> setTargetAction(TargetAction.UA)));
    controlBoard
        .setGoalCoralStation()
        .onTrue(new InstantCommand(() -> setTargetAction(TargetAction.HP)));
    controlBoard
        .setGoalAlgaeScoring()
        .onTrue(new InstantCommand(() -> setTargetAction(TargetAction.AS)));
    controlBoard.setGoalTravel().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.TR)));
    controlBoard.homeElevator().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.HM)));

    // controlBoard
    //     .reefAB()
    //     .onTrue(new InstantCommand(() -> setTargetReefSide(TargetFieldLocation.AB)));
    // controlBoard
    //     .reefCD()
    //     .onTrue(new InstantCommand(() -> setTargetReefSide(TargetFieldLocation.CD)));
    // controlBoard
    //     .reefEF()
    //     .onTrue(new InstantCommand(() -> setTargetReefSide(TargetFieldLocation.EF)));
    // controlBoard
    //     .reefGH()
    //     .onTrue(new InstantCommand(() -> setTargetReefSide(TargetFieldLocation.GH)));
    // controlBoard
    //     .reefIJ()
    //     .onTrue(new InstantCommand(() -> setTargetReefSide(TargetFieldLocation.IJ)));
    // controlBoard
    //     .reefKL()
    //     .onTrue(new InstantCommand(() -> setTargetReefSide(TargetFieldLocation.KL)));

    // controlBoard
    //     .setSubReefLeft()
    //     .onTrue(new InstantCommand(() -> setReefSubSide(ReefSubSide.LEFT)));
    // controlBoard
    //     .setSubReefCenter()
    //     .onTrue(new InstantCommand(() -> setReefSubSide(ReefSubSide.CENTER)));
    // controlBoard
    //     .setSubReefRight()
    //     .onTrue(new InstantCommand(() -> setReefSubSide(ReefSubSide.RIGHT)));
  }

  public void setTargetReefSide(TargetFieldLocation target) {
    targetReefSide = target;
    revealCombination();
  }

  public void setTargetAction(TargetAction target) {
    targetAction = target;
    revealCombination();
  }

  public void setReefSubSide(ReefSubSide target) {
    reefSubSide = target;
    revealCombination();
  }

  public TargetFieldLocation getTargetReefSide() {
    return targetReefSide;
  }

  public TargetAction getTargetAction() {
    return targetAction;
  }

  public ReefSubSide getReefSubSide() {
    return reefSubSide;
  }

  public void revealCombination() {
    System.out.println(
        "Targeting : "
            + getTargetReefSide().toString()
            + " at "
            + getReefSubSide().toString()
            + " with action "
            + getTargetAction().toString());
  }

  public enum TargetFieldLocation {
    AB(7, 18, Degrees.of(0)),
    CD(8, 17, Degrees.of(60)),
    EF(9, 22, Degrees.of(120)),
    GH(10, 21, Degrees.of(180)),
    IJ(11, 20, Degrees.of(240)),
    KL(6, 19, Degrees.of(300)),
    RCS(2, 12, Degrees.of(-306)),
    LCS(1, 13, Degrees.of(306)),
    PRC(3, 16, Degrees.of(0)), // Processor (side of the feild, not where the teams human player is)
    FBG(
        15,
        4,
        Degrees.of(
            -90)), // Far Side of Barge from teams driver station (so if on blue, the red tag will
    // be on the near side)
    NBG(14, 0, Degrees.of(90)); // Same as above but for the near side

    public final int redTagID;
    public final int blueTagID;
    public final Angle lineupAngle;

    private TargetFieldLocation(int redTagID, int blueTagID, Angle lineupAngle) {
      this.redTagID = redTagID;
      this.blueTagID = blueTagID;
      this.lineupAngle = lineupAngle;
    }

    public int getRedTagID() {
      return redTagID;
    }

    public int getBlueTagID() {
      return blueTagID;
    }

    public Angle getLineupAngle() {
      return lineupAngle; 
    }
  }

  public enum TargetAction {
    HM(1.0, Degrees.of(170.0), Degrees.of(90)), // Homing
    L1L(1.0, Degrees.of(255.0), Degrees.of(90)),
    L1H(1.0, Degrees.of(240.0), Degrees.of(90)),
    L2(21.0, Degrees.of(287.0), Degrees.of(90)),
    L3(38.5, Degrees.of(287.0), Degrees.of(90)),
    L4(63.0, Degrees.of(283.0), Degrees.of(90)),
    LA(1.0, Degrees.of(170.0), Degrees.of(18)), // Lower Algae
    UA(27.0, Degrees.of(170.0), Degrees.of(90)), // Upper Algae
    HP(1.0, Degrees.of(110), Degrees.of(90)), // Coral Station
    AS(55.0, Degrees.of(180.0), Degrees.of(135)), // Algae Scoring
    TR(5.0, Degrees.of(180.0), Degrees.of(90)); // Travel

    public final double ElevatorPositionRotations;
    public final Angle coralArmAngle;
    public final Angle algaeArmAngle;

    private TargetAction(double positionRotations, Angle coralArmAngle, Angle algaeArmAngle) {
      this.ElevatorPositionRotations = positionRotations;
      this.coralArmAngle = coralArmAngle;
      this.algaeArmAngle = algaeArmAngle;
    }

    public double getElevatorPositionRotations() {
      return ElevatorPositionRotations;
    }

    public Angle getCoralArmAngle() {
      return coralArmAngle;
    }

    public Angle getAlgaeArmPosition() {
      return algaeArmAngle;
    }
  }

  public enum ReefSubSide {
    LEFT(new Transform2d(0.5, 0.25, new Rotation2d(0))),
    CENTER(new Transform2d(0.5, 0.0, new Rotation2d(0))),
    RIGHT(new Transform2d(0.5, -0.25, new Rotation2d(0)));

    public Transform2d transform;

    private ReefSubSide(Transform2d gt) {
      this.transform = gt;
    }
  }
}
