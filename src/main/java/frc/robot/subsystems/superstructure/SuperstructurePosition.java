package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.SuperstructureConstants;

public class SuperstructurePosition {
    public enum TargetAction {
        // spotless: off
        HM(0.5, Degrees.of(228.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Homing
        EXPLODE(5.0, Degrees.of(306.5), IntakePivotPositions.STOW.position, ActionType.NONE),
        INTAKE(3.0, Degrees.of(305), IntakePivotPositions.INTAKE.position, ActionType.CORAL),
        L1H(6.0, Degrees.of(190.0), IntakePivotPositions.L1.position, ActionType.CORAL),
        L2(17.7, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL),
        L3(36.0, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL),
        L4(62.5, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL),
        LA(6.0, Degrees.of(265.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Lower Algae
        UA(23.0, Degrees.of(265.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Upper Algae
        SAFE_ARM_HEIGHT(
                SuperstructureConstants.MIN_SAFE_ROTATION,
                Degrees.of(228),
                IntakePivotPositions.STOW.position,
                ActionType.NONE),
        STOW(12.0, Degrees.of(140.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Coral Station
        AS(65.0, Degrees.of(175.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Algae Scoring NET
        AP(5.0, Degrees.of(325.0), IntakePivotPositions.MID.position, ActionType.ALGAE), // Algae Scoring Processor
        TR(5.0, Degrees.of(228.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Travel
        CL(14.0, Degrees.of(265.0), IntakePivotPositions.MID.position, ActionType.NONE); // Climb

        // spotless: on
        private final double elevatorPosition;
        private final Angle armPivotAngle;
        private final double intakePosition;
        private final ActionType type;

        private TargetAction(double elevatorPosition, Angle coralArmAngle, double intakePosition, ActionType type) {
            this.elevatorPosition = elevatorPosition;
            this.armPivotAngle = coralArmAngle;
            this.intakePosition = intakePosition;
            this.type = type;
        }

        public double getElevatorPositionRotations() {
            return elevatorPosition;
        }

        public Angle getArmPivotAngle() {
            return armPivotAngle;
        }

        public double getIntakePivotPosition() {
            return intakePosition;
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

    private enum IntakePivotPositions {
        STOW(18),
        INTAKE(0.22),
        L1(15),
        MID(9);

        private double position;

        private IntakePivotPositions(double position) {
            this.position = position;
        }
    }
}
