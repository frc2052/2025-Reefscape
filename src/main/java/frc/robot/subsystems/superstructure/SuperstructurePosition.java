package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.SuperstructureConstants;

public class SuperstructurePosition {
    public enum TargetAction {
        HM(0.5, Degrees.of(228.0), Degrees.of(135), ActionType.NONE), // Homing
        INTAKE(6.0, Degrees.of(300.0), Degrees.of(30), ActionType.CORAL),
        L1H(6.0, Degrees.of(190.0), Degrees.of(135), ActionType.CORAL),
        L2(21.5, Degrees.of(190.0), Degrees.of(135), ActionType.CORAL),
        L3(31.5, Degrees.of(190.0), Degrees.of(135), ActionType.CORAL),
        L4(65.0, Degrees.of(190.0), Degrees.of(135), ActionType.CORAL),
        LA(6.0, Degrees.of(265.0), Degrees.of(135), ActionType.ALGAE), // Lower Algae
        UA(23.0, Degrees.of(265.0), Degrees.of(135), ActionType.ALGAE), // Upper Algae
        MIN_ARM(SuperstructureConstants.UPWARDS_MIN_ELEVATOR, Degrees.of(228), Degrees.of(135), ActionType.NONE),
        STOW(3.0, Degrees.of(221), Degrees.of(135), ActionType.NONE), // Coral Station
        AS(65.0, Degrees.of(175.0), Degrees.of(135), ActionType.ALGAE), // Algae Scoring NET
        AP(5.0, Degrees.of(325.0), Degrees.of(135), ActionType.ALGAE), // Algae Scoring Processor
        TR(5.0, Degrees.of(228.0), Degrees.of(135), ActionType.NONE), // Travel
        CL(8.0, Degrees.of(228.0), Degrees.of(135), ActionType.NONE); // Climb

        private final double elevatorPosition;
        private final Angle armPivotAngle;
        private final Angle intakePivotAngle;
        private final ActionType type;

        private TargetAction(double elevatorPosition, Angle coralArmAngle, Angle algaeArmAngle, ActionType type) {
            this.elevatorPosition = elevatorPosition;
            this.armPivotAngle = coralArmAngle;
            this.intakePivotAngle = algaeArmAngle;
            this.type = type;
        }

        public double getElevatorPositionRotations() {
            return elevatorPosition;
        }

        public Angle getArmPivotAngle() {
            return armPivotAngle;
        }

        public Angle getIntakePivotPosition() {
            return intakePivotAngle;
        }

        public ActionType getType() {
            return type;
        }
    }

    public enum ActionType {
        NONE,
        CORAL,
        ALGAE
    }
}
