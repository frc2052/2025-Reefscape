package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class SuperstructurePosition {
  public enum TargetAction {
    HM(1.0, Degrees.of(170.0), Degrees.of(90), ActionType.NONE), // Homing
    L1L(1.0, Degrees.of(255.0), Degrees.of(90), ActionType.CORAL),
    L1H(1.0, Degrees.of(240.0), Degrees.of(90), ActionType.CORAL),
    L2(21.0, Degrees.of(287.0), Degrees.of(90), ActionType.CORAL),
    L3(38.5, Degrees.of(287.0), Degrees.of(90), ActionType.CORAL),
    L4(63.0, Degrees.of(277.5), Degrees.of(90), ActionType.CORAL),
    LA(1.0, Degrees.of(170.0), Degrees.of(18), ActionType.ALGAE), // Lower Algae
    UA(27.0, Degrees.of(170.0), Degrees.of(90), ActionType.ALGAE), // Upper Algae
    HP(1.0, Degrees.of(113), Degrees.of(90), ActionType.STATION), // Coral Station
    AS(55.0, Degrees.of(180.0), Degrees.of(135), ActionType.NONE), // Algae Scoring NET
    AP(55.0, Degrees.of(180.0), Degrees.of(135), ActionType.PROCESS), // Algae Scoring Processor
    TR(5.0, Degrees.of(180.0), Degrees.of(90), ActionType.NONE), // Travel
    CL(13.0, Degrees.of(170.0), Degrees.of(90.0), ActionType.NONE); // Climb

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
}
