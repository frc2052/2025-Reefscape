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

    private static TargetReefSide targetReefSide = TargetReefSide.AB;
    private static TargetAction targetAction = TargetAction.HP;
    private static ReefSubSide reefSubSide = ReefSubSide.CENTER;

    public static PositionSuperstructure getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PositionSuperstructure();
        }
        return INSTANCE;
    }

    private PositionSuperstructure() {
        controlBoard.setGoalL1().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L1)));
        controlBoard.setGoalL2().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L2)));
        controlBoard.setGoalL3().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L3)));
        controlBoard.setGoalL4().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.L4)));
        controlBoard.setGoalLowerAlgae().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.LA)));
        controlBoard.setGoalUpperAlgae().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.UA)));
        controlBoard.setGoalCoralStation().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.HP)));
        controlBoard.setGoalAlgaeScoring().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.AS)));
        controlBoard.setGoalTravel().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.TR)));
        controlBoard.homeElevator().onTrue(new InstantCommand(() -> setTargetAction(TargetAction.HM)));

        controlBoard.reefAB().onTrue(new InstantCommand(() -> setTargetReefSide(TargetReefSide.AB)));
        controlBoard.reefCD().onTrue(new InstantCommand(() -> setTargetReefSide(TargetReefSide.CD)));
        controlBoard.reefEF().onTrue(new InstantCommand(() -> setTargetReefSide(TargetReefSide.EF)));
        controlBoard.reefGH().onTrue(new InstantCommand(() -> setTargetReefSide(TargetReefSide.GH)));
        controlBoard.reefIJ().onTrue(new InstantCommand(() -> setTargetReefSide(TargetReefSide.IJ)));
        controlBoard.reefKL().onTrue(new InstantCommand(() -> setTargetReefSide(TargetReefSide.KL)));

        controlBoard.setSubReefLeft().onTrue(new InstantCommand(() -> setReefSubSide(ReefSubSide.LEFT)));
        controlBoard.setSubReefCenter().onTrue(new InstantCommand(() -> setReefSubSide(ReefSubSide.CENTER)));
        controlBoard.setSubReefRight().onTrue(new InstantCommand(() -> setReefSubSide(ReefSubSide.RIGHT)));
    }

    public static void setTargetReefSide(TargetReefSide target) {
        targetReefSide = target;
        revealCombonation();
    }

    public static void setTargetAction(TargetAction target) {
        targetAction = target;
        revealCombonation();
    }

    public static void setReefSubSide(ReefSubSide target) {
        reefSubSide = target;
        revealCombonation();
    }

    public static TargetReefSide getTargetReefSide() {
        return targetReefSide;
    }

    public static TargetAction getTargetAction() {
        return targetAction;
    }

    public static ReefSubSide getReefSubSide() {
        return reefSubSide;
    }

    public static void revealCombonation() {
        System.out.println("Targeting : " + getTargetReefSide().toString() + " at " + getReefSubSide().toString() + " with action " + getTargetAction().toString());
    }

    public static enum TargetReefSide {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL
    }

    public static enum TargetAction {
        HM(2.0,  Degrees.of(180.0), Degrees.of(90)), // Homing
        L1(10.0, Degrees.of(110.0), Degrees.of(90)),
        L2(20.0, Degrees.of(55.0),  Degrees.of(90)),
        L3(37.5, Degrees.of(55.0),  Degrees.of(90)),
        L4(55.0, Degrees.of(75.0),  Degrees.of(90)),
        LA(25.0, Degrees.of(90.0),  Degrees.of(90)), // Lower Algae
        UA(27.0, Degrees.of(90.0),  Degrees.of(90)), // Upper Algae
        HP(7.0,  Degrees.of(300),   Degrees.of(90)), // Coral Station
        AS(55.0, Degrees.of(180.0), Degrees.of(135)), // Algae Scoring
        TR(5.0,  Degrees.of(180.0), Degrees.of(90)); // Travel

        public final double ElevatorPositionRotations;
        public final Angle coralArmAngle;
        public final Angle algaeArmAngle;

        private TargetAction(double positionRotations, Angle armAngle, Angle algaeArmAngle) {
            this.ElevatorPositionRotations = positionRotations;
            this.coralArmAngle = armAngle;
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

    public static enum ReefSubSide {
        LEFT(new Transform2d(0.5, 0.25, new Rotation2d(0))),
        CENTER(new Transform2d(0.5, 0.0, new Rotation2d(0))),
        RIGHT(new Transform2d(0.5, -0.25, new Rotation2d(0)));

        public Transform2d transform;

        private ReefSubSide(Transform2d gt) {
        this.transform = gt;
        }
    }
}

