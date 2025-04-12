package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.RobotState;

public class SuperstructurePosition {
    public enum TargetAction {
        // spotless:off
        HM(1.1, Degrees.of(130.0), IntakePivotPositions.MID.position, ActionType.NONE), // Homing
        HP(0.75, Degrees.of(133.0), IntakePivotPositions.MID.position, ActionType.NONE),
        EXPLODE(3.75, Degrees.of(130.5), IntakePivotPositions.STOW.position, ActionType.NONE),
        INTAKE(1.5, Degrees.of(307), IntakePivotPositions.INTAKE.position, ActionType.CORAL), //307
        UN_JAM(19, Degrees.of(150.0), IntakePivotPositions.INTAKE.position, ActionType.CORAL),
        L1H(4.5, Degrees.of(190.0), IntakePivotPositions.L1.position, ActionType.CORAL),
        L2(13.275, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL, 3.0, Degrees.of(158.0), IntakePivotPositions.STOW.position),
        L3(27.0, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL, 15.25, Degrees.of(154.0), IntakePivotPositions.STOW.position),
        L4(47.5, Degrees.of(196.0), IntakePivotPositions.STOW.position, ActionType.CORAL, 47.5, Degrees.of(186.0), IntakePivotPositions.STOW.position),
        LA(3.5, Degrees.of(265.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Lower Algae
        UA(17.25, Degrees.of(265.0), IntakePivotPositions.STOW.position, ActionType.ALGAE), // Upper Algae
        SAFE_ARM_HEIGHT(SuperstructureConstants.MIN_SAFE_ROTATION, Degrees.of(228), IntakePivotPositions.STOW.position, ActionType.NONE),
        STOW(9.0, Degrees.of(130.0), IntakePivotPositions.STOW.position, ActionType.NONE), // default state
        POST_ALGAE_STOW(9.0, Degrees.of(305.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Coral Station
        AS(46, Degrees.of(190.0), IntakePivotPositions.MID.position, ActionType.ALGAE), // Algae Scoring NET
        AP(2.25, Degrees.of(325.0), IntakePivotPositions.MID.position, ActionType.ALGAE), // Algae Scoring Processor
        // GROUND_ALGAE_INTAKE(9.0, Degrees.of(130.0), IntakePivotPositions.GROUND_ALGAE.position, ActionType.ALGAE),
        GROUND_ALGAE_INTAKE(0.0, Degrees.of(322.0), IntakePivotPositions.MID.position, ActionType.ALGAE),
        GROUND_ALGAE_HANDOFF(13.0, Degrees.of(100.0), IntakePivotPositions.ALGAE_HANDOFF.position, ActionType.ALGAE),
        TR(3.75, Degrees.of(228.0), IntakePivotPositions.STOW.position, ActionType.NONE), // Travel
        CL(10.5, Degrees.of(265.0), IntakePivotPositions.L1.position, ActionType.NONE); // Climb

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
        INTAKE(0.8), // was 75
        L1(15),
        MID(7),
        GROUND_ALGAE(3.5),
        ALGAE_HANDOFF(17.5);

        private double position;

        private IntakePivotPositions(double position) {
            this.position = position;
        }
    }
}
