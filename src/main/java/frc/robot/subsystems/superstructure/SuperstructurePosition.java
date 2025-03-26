package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.RobotState;

public class SuperstructurePosition {
    public enum TargetAction {
        // spotless:off
        HM(1.5, Degrees.of(228.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Homing
        EXPLODE(5.0, Degrees.of(228.5), IntakePivotPositions.STOW.position, ActionType.NONE),
        INTAKE(2.0, Degrees.of(307), IntakePivotPositions.INTAKE.position, ActionType.CORAL),
        L1H(6.0, Degrees.of(190.0), IntakePivotPositions.L1.position, ActionType.CORAL),
        L2(17.7, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL, 4.0, Degrees.of(157.0), IntakePivotPositions.STOW.position),
        L3(36.0, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL, 21.0, Degrees.of(157.0), IntakePivotPositions.STOW.position),
        L4(62.5, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL, 62.5, Degrees.of(186.0), IntakePivotPositions.STOW.position),
        LA(6.0, Degrees.of(265.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Lower Algae
        UA(23.0, Degrees.of(265.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Upper Algae
        SAFE_ARM_HEIGHT(SuperstructureConstants.MIN_SAFE_ROTATION, Degrees.of(228), IntakePivotPositions.STOW.position, ActionType.NONE),
        STOW(12.0, Degrees.of(130.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Coral Station
        POST_ALGAE_STOW(12.0, Degrees.of(305.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Coral Station
        AS(65.0, Degrees.of(185.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Algae Scoring NET
        AP(5.0, Degrees.of(325.0), IntakePivotPositions.MID.position, ActionType.ALGAE), // Algae Scoring Processor
        TR(5.0, Degrees.of(228.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Travel
        CL(14.0, Degrees.of(265.0), IntakePivotPositions.MID.position, ActionType.NONE); // Climb

        // spotless:on
        private final double elevatorPosition;
        private final Angle armPivotAngle;
        private final double intakePosition;
        private final ActionType type;
        private final double elevatorPosition1c;
        private final Angle armPivotAngle1c;
        private final double intakePosition1c;

        private TargetAction(double elevatorPosition, Angle armPivotAngle, double intakePosition, ActionType type) {
            this.elevatorPosition = elevatorPosition;
            this.armPivotAngle = armPivotAngle;
            this.intakePosition = intakePosition;
            this.elevatorPosition1c = elevatorPosition;
            this.armPivotAngle1c = armPivotAngle;
            this.intakePosition1c = intakePosition;
            this.type = type;
        }

        private TargetAction(
                double elevatorPosition,
                Angle armPivotAngle,
                double intakePosition,
                ActionType type,
                double elevatorPosition1c,
                Angle armPivotAngle1c,
                double intakePosition1c) {
            this.elevatorPosition = elevatorPosition;
            this.armPivotAngle = armPivotAngle;
            this.intakePosition = intakePosition;
            this.elevatorPosition1c = elevatorPosition1c;
            this.armPivotAngle1c = armPivotAngle1c;
            this.intakePosition1c = intakePosition1c;
            this.type = type;
        }

        public double getElevatorPositionRotations() {
            return RobotState.getInstance().getIsFlushAlign() ? elevatorPosition : elevatorPosition1c;
        }

        public Angle getArmPivotAngle() {
            return RobotState.getInstance().getIsFlushAlign() ? armPivotAngle : armPivotAngle1c;
        }

        public double getIntakePivotPosition() {
            return RobotState.getInstance().getIsFlushAlign() ? intakePosition : intakePosition1c;
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
        STOW(17.5),
        INTAKE(0.75),
        L1(15),
        MID(9);

        private double position;

        private IntakePivotPositions(double position) {
            this.position = position;
        }
    }
}
