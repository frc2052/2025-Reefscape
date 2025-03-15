package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class SuperstructurePosition {
    public enum TargetAction {
        HM(0.5, Degrees.of(170.0), Degrees.of(135)), // Homing
        INTAKE(6.0, Degrees.of(300.0), Degrees.of(30)),
        L1H(6.0, Degrees.of(190.0), Degrees.of(135)),
        L2(21.5, Degrees.of(190.0), Degrees.of(135)),
        L3(31.5, Degrees.of(190.0), Degrees.of(135)),
        L4(62.5, Degrees.of(200), Degrees.of(135)),
        LA(6.0, Degrees.of(265.0), Degrees.of(135)), // Lower Algae
        UA(40.0, Degrees.of(265.0), Degrees.of(135)), // Upper Algae
        STOW(1.5, Degrees.of(228), Degrees.of(135)), // Coral Station
        AS(55.0, Degrees.of(252.0), Degrees.of(135)), // Algae Scoring NET
        AP(55.0, Degrees.of(325.0), Degrees.of(135)), // Algae Scoring Processor
        TR(5.0, Degrees.of(228.0), Degrees.of(135)), // Travel
        CL(8.0, Degrees.of(228.0), Degrees.of(135)); // Climb

        private final double elevatorPosition;
        private final Angle armPivotAngle;
        private final Angle intakePivotAngle;

        private TargetAction(double elevatorPosition, Angle coralArmAngle, Angle algaeArmAngle) {
            this.elevatorPosition = elevatorPosition;
            this.armPivotAngle = coralArmAngle;
            this.intakePivotAngle = algaeArmAngle;
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
    }
}
