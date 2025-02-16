package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;

public class SuperstructurePosition {
  public enum TargetFieldLocation {
    AB(7, 18, Degrees.of(0)),
    CD(8, 17, Degrees.of(60)),
    EF(9, 22, Degrees.of(120)),
    GH(10, 21, Degrees.of(180)),
    IJ(11, 20, Degrees.of(240)),
    KL(6, 19, Degrees.of(300)),
    RCS(2, 12, Degrees.of(-306)),
    LCS(1, 13, Degrees.of(306)),
    PRC(3, 16, Degrees.of(0)), // Processor (side of the field, not where the teams human player is)
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

    public int getTagID() {
      return RobotState.getInstance().isRedAlliance() ? redTagID : blueTagID;
    }

    public Angle getLineupAngle() {
      return lineupAngle;
    }
  }

  public enum TargetAction {
    HM(1.0, Degrees.of(170.0), Degrees.of(90), ActionType.NONE), // Homing
    L1L(1.0, Degrees.of(255.0), Degrees.of(90), ActionType.CORAL),
    L1H(1.0, Degrees.of(240.0), Degrees.of(90), ActionType.CORAL),
    L2(21.0, Degrees.of(287.0), Degrees.of(90), ActionType.CORAL),
    L3(38.5, Degrees.of(287.0), Degrees.of(90), ActionType.CORAL),
    L4(63.0, Degrees.of(283.0), Degrees.of(90), ActionType.CORAL),
    LA(1.0, Degrees.of(170.0), Degrees.of(18), ActionType.ALGAE), // Lower Algae
    UA(27.0, Degrees.of(170.0), Degrees.of(90), ActionType.ALGAE), // Upper Algae
    HP(1.0, Degrees.of(110), Degrees.of(90), ActionType.STATION), // Coral Station
    AS(55.0, Degrees.of(180.0), Degrees.of(135), ActionType.NONE), // Algae Scoring NET
    AP(55.0, Degrees.of(180.0), Degrees.of(135), ActionType.PROCESS), // Algae Scoring Processor
    TR(5.0, Degrees.of(180.0), Degrees.of(90), ActionType.NONE); // Travel

    private final double elevatorPosition;
    private final Angle coralArmAngle;
    private final Angle algaeArmAngle;
    private final ActionType actionType;

    private TargetAction(
        double elevatorPosition, Angle coralArmAngle, Angle algaeArmAngle, ActionType actionType) {
      this.elevatorPosition = elevatorPosition;
      this.coralArmAngle = coralArmAngle;
      this.algaeArmAngle = algaeArmAngle;
      this.actionType = actionType;
    }

    public double getElevatorPositionRotations() {
      return elevatorPosition;
    }

    public Angle getCoralArmAngle() {
      return coralArmAngle;
    }

    public Angle getAlgaeArmPosition() {
      return algaeArmAngle;
    }

    public ActionType getActionType() {
      return actionType;
    }
  }

  public enum ActionType {
    CORAL,
    ALGAE,
    STATION,
    PROCESS,
    NONE
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
