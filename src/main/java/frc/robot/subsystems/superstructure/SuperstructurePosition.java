package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class SuperstructurePosition {
    public enum TargetAction {
        HM(0.5, Degrees.of(170.0), Degrees.of(135), ActionType.NONE), // Homing
        // L1L(1.0, Degrees.of(255.0), Degrees.of(90), ActionType.CORAL),
        L1H(1.0, Degrees.of(240.0), Degrees.of(135), ActionType.CORAL),
        L2(21.5, Degrees.of(287.0), Degrees.of(135), ActionType.CORAL),
        L3(38.5, Degrees.of(287.0), Degrees.of(135), ActionType.CORAL),
        L4(62.5, Degrees.of(279), Degrees.of(135), ActionType.CORAL),
        LA(6.0, Degrees.of(170.0), Degrees.of(135), ActionType.ALGAE), // Lower Algae
        UA(40.0, Degrees.of(170.0), Degrees.of(135), ActionType.ALGAE), // Upper Algae
        HP(1.0, Degrees.of(111), Degrees.of(135), ActionType.STATION), // Coral Station
        AS(55.0, Degrees.of(180.0), Degrees.of(135), ActionType.NONE), // Algae Scoring NET
        AP(55.0, Degrees.of(180.0), Degrees.of(135), ActionType.PROCESS), // Algae Scoring Processor
        TR(5.0, Degrees.of(180.0), Degrees.of(135), ActionType.NONE), // Travel
        CL(13.0, Degrees.of(170.0), Degrees.of(135), ActionType.NONE); // Climb

        private final double elevatorPosition;
        private final Angle coralArmAngle;
        private final Angle algaeArmAngle;
        private final ActionType actionType;

        private TargetAction(double elevatorPosition, Angle coralArmAngle, Angle algaeArmAngle, ActionType actionType) {
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

        public Angle getAlgaeArmPivotPosition() {
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
