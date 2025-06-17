package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import com.team2052.lib.util.SecondaryImageManager;
import frc.robot.RobotContainer;
import com.team2052.lib.util.SecondaryImageManager.SecondaryImage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldLocation;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Dashboard;
import java.util.function.Supplier;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSubsystem extends SubsystemBase {

    private static SuperstructureSubsystem INSTANCE;

    private Supplier<Double> elevatorNudgeSupplier =
            () -> Dashboard.getInstance().getElevatorNudgeValue();
    private static LoggedNetworkString elevatorNudgeDisplay =
            new LoggedNetworkString(DashboardConstants.ELVATOR_NUDGE_VALUE_DISPLAY_KEY, "DEFAULT - 0.0");

    private static LoggedNetworkBoolean elevatorNudgeSaved =
            new LoggedNetworkBoolean(DashboardConstants.ELEVATOR_NUDGE_SAVED_KEY, false);

    private RobotState robotState = RobotState.getInstance();
    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private ArmPivotSubsystem armPivot = ArmPivotSubsystem.getInstance();
    private IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();

    private TargetAction selectedTargetAction = TargetAction.EXPLODE;
    private TargetAction currentAction = TargetAction.EXPLODE;

    private TargetAction previousAction;
    private boolean isChangingState;

    private boolean cancelHome = false;
    private boolean driverAction;
    private boolean shouldSmartDrive;
    private boolean algaeScoreDownNeeded = false;
    private boolean movingFromIntake = false;

    private double selectedElevatorNudge;
    private double savedElevatorNudge;

    /** Private constructor to prevent instantiation. */
    private SuperstructureSubsystem() {
        previousAction = getSelectedTargetAction();
        // pushChangedValueToShuffleboard(previousAction);
        isChangingState = false;
    }

    /** Public method to provide access to the instance. */
    public static SuperstructureSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SuperstructureSubsystem();
        }
        return INSTANCE;
    }

    public boolean elevatorNudgeRecompileNeeded() {
        return elevatorNudgeSupplier.get().doubleValue() != savedElevatorNudge;
    }

    public void elevatorNudgeRecompile() {
        elevatorNudgeSaved.set(false);
        selectedElevatorNudge = elevatorNudgeSupplier.get().doubleValue();
        savedElevatorNudge = selectedElevatorNudge;
        elevatorNudgeDisplay.set("Elevator nudge: " + savedElevatorNudge);
        elevatorNudgeSaved.set(true);
    }

    public double getElevatorNudgeValue() {
        return savedElevatorNudge;
    }

    public boolean isAtTargetState() {
        return isChangingState;
    }

    public TargetAction getLatestAction() {
        return previousAction;
    }

    // private void pushChangedValueToShuffleboard(TargetAction action) {
    //     switch (action) {
    //         case L1H:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L1);
    //             break;
    //         case L2:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L2);
    //             break;
    //         case L3:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L3);
    //             break;
    //         case L4:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.L4);
    //             break;
    //         case UPPER_ALGAE:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.A2);
    //             break;
    //         case LOWER_ALGAE:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.A1);
    //             break;
    //         default:
    //             SecondaryImageManager.setCurrentImage(SecondaryImage.NONE);
    //             break;
    //     }
    // }

    public Command set(TargetAction target, boolean confirm) {
        return new InstantCommand(() -> setSelectedTargetAction(target, confirm));
    }

    public void setSelectedTargetAction(TargetAction target, boolean confirm) {
        selectedTargetAction = target;
        // pushChangedValueToShuffleboard(selectedTargetAction);
        if (confirm) {
            confirmSelectedAction();
        }
        revealCombination();
    }

    public void stow() {
        setCurrentAction(getStowFromCurrent());
    }

    public void setCurrentAction(TargetAction target) {
        currentAction = target;
        driverAction = true;
    }

    private void setSmartDriveAction(TargetAction target) {
        currentAction = target;
        driverAction = false;
    }

    public Command confirm() {
        return new InstantCommand(() -> confirmSelectedAction());
    }

    public void confirmSelectedAction() {
        currentAction = selectedTargetAction;
        movingFromIntake = false;
        revealCombination();
    }

    public TargetAction getSelectedTargetAction() {
        return selectedTargetAction;
    }

    public TargetAction getCurrentAction() {
        return currentAction;
    }

    public void revealCombination() {
        System.out.println("Goal : " + getSelectedTargetAction().toString());
    }

    @Override
    public void periodic() {
        TargetAction goalTargetAction = getCurrentAction();
        Logger.recordOutput("Superstructure/Current = Selected", goalTargetAction == getSelectedTargetAction());
        Logger.recordOutput("Target Superstructure Changing State", isChangingState);

        if (goalTargetAction != previousAction) {
            Logger.recordOutput("Target Superstructure State Has Changed", true);
            isChangingState = true;
            if (goalTargetAction != TargetAction.HOME) {
                cancelHome = true;
            }
        } else {
            Logger.recordOutput("Target Superstructure State Has Changed", false);
        }

        if (goalTargetAction == TargetAction.ALGAE_NET) {
            algaeScoreDownNeeded = true;
        }

        if (IntakeRollerSubsystem.getInstance().isHoldingCoral()) {
            setCurrentAction(TargetAction.TRAVEL);
        } else if (!movingFromIntake
                && RobotState.getInstance().getHasCoral()
                && (goalTargetAction == TargetAction.INTAKE)
                && armPivot.atPosition(goalTargetAction)) {
            movingFromIntake = true;
            setCurrentAction(DriverStation.isAutonomous() ? TargetAction.L3 : TargetAction.STOW);
        }

        if (armPivot.atPosition(TargetAction.STOW)) {
            movingFromIntake = false;
        }

        if (isChangingState) {
            if (cancelHome) {
                elevator.setWantHome(false);
                cancelHome = false;
            } else if (goalTargetAction == TargetAction.HOME && !elevator.isHoming()) {
                elevator.setWantHome(true);
                intakePivot.setAngle(TargetAction.HOME.getIntakePivotPosition());
                armPivot.setArmPosition(TargetAction.HOME);
                System.out.println("HOMING");
                return;
            }

            boolean armCrossingDanger = willArmCrossDangerZone(
                    armPivot.getArmAngle().in(Degrees),
                    goalTargetAction.getArmPivotAngle().in(Degrees));

            if (goalTargetAction.getElevatorPositionRotations() < SuperstructureConstants.MIN_SAFE_ROTATION
                    && elevator.getPosition() < SuperstructureConstants.MIN_SAFE_ROTATION
                    && armCrossingDanger) {
                intakePivot.setPosition(goalTargetAction);

                if (!armPivot.isAtPosition(5, goalTargetAction.getArmPivotAngle())) {
                    elevator.setPositionMotionMagic(TargetAction.SAFE_ARM_HEIGHT);
                }

                if (elevator.atPosition(1.0, TargetAction.SAFE_ARM_HEIGHT)) {
                    armPivot.setArmPosition(goalTargetAction);
                    if (armPivot.isAtPosition(5, goalTargetAction.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(goalTargetAction);
                    }
                }
            } else if (goalTargetAction.getElevatorPositionRotations() < elevator.getPosition()) {
                if (algaeScoreDownNeeded) {
                    armPivot.setArmPosition(goalTargetAction);
                    intakePivot.setPosition(goalTargetAction);

                    if (armPivot.isAtPosition(3, goalTargetAction.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(goalTargetAction);
                        algaeScoreDownNeeded = false;
                    }
                }
                if (goalTargetAction.getElevatorPositionRotations() > SuperstructureConstants.MIN_SAFE_ROTATION
                        || elevator.getPosition() > SuperstructureConstants.MIN_MOVE_ROTATION) {
                    elevator.setPositionMotionMagic(goalTargetAction);
                    armPivot.setArmPosition(goalTargetAction);
                    intakePivot.setPosition(goalTargetAction);
                } else {
                    armPivot.setArmPosition(goalTargetAction);
                    intakePivot.setPosition(goalTargetAction);

                    if (armPivot.isAtPosition(10, goalTargetAction.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(goalTargetAction);
                    }
                }
            } else if (goalTargetAction.getElevatorPositionRotations() > elevator.getPosition()) {
                elevator.setPositionMotionMagic(goalTargetAction);
                intakePivot.setPosition(goalTargetAction);

                if (elevator.atPosition(20, goalTargetAction)) {
                    armPivot.setArmPosition(goalTargetAction);
                }
            }

            if (elevator.atPosition(goalTargetAction)
                    && armPivot.isAtPosition(5, goalTargetAction.getArmPivotAngle())) {
                isChangingState = false;
                Logger.recordOutput("Arrived at Target State", true);
            } else {
                Logger.recordOutput("Arrived at Target State", false);
            }
        }

        previousAction = goalTargetAction;
        // else {
        //     if (RobotState.getInstance().getHasCoral()
        //             && (getCurrentAction() == TargetAction.STOW)
        //             && !RobotState.getInstance().getIsIntaking()) {
        //         if (getSelectedTargetAction() == TargetAction.L2
        //                 || getSelectedTargetAction() == TargetAction.L3
        //                 || getSelectedTargetAction() == TargetAction.L4) {
        //             setCurrentAction(TargetAction.L2);
        //         } else if (getSelectedTargetAction() == TargetAction.L1H
        //                 && (getCurrentAction() == TargetAction.STOW)
        //                 && !RobotState.getInstance().getIsIntaking()) {
        //             setCurrentAction(TargetAction.L1H);
        //         } else if ((getCurrentAction() == TargetAction.STOW)
        //                 && !RobotState.getInstance().getIsIntaking()) {
        //             setCurrentAction(TargetAction.TR);
        //         }
        //     }
        // }

        previousAction = target;
        setDriverAction(driverAction);
        setTargetAction();
        System.out.println("----------------------------------------------------- shouldSmartDrive " + shouldSmartDrive
                + " Driver action " + driverAction);

        Logger.recordOutput(
                "SmartDrive/current position",
                SuperstructureSubsystem.getInstance().getCurrentAction());
        Logger.recordOutput("SmartDrive/should smart drive", shouldSmartDrive);
        Logger.recordOutput("SmartDrive/ driver action", driverAction);
    }

    private void setTargetAction() {
        // getCurrentAction() == TargetAction.STOW || getCurrentAction() == TargetAction.TR &&
        if (shouldSmartDrive && RobotContainer.getSmartDrivewanted()) {
            if (RobotState.getFieldLocation() == FieldLocation.REEF
                    && IntakeRollerSubsystem.getInstance().getHasCoral()) {// replace IntakeRollerSubsystem with RobotState. 
                System.out.println("inside the zone");
                if (robotState.getAlignOffset() == AlignOffset.LEFT_BRANCH
                        || robotState.getAlignOffset() == AlignOffset.RIGHT_BRANCH) {
                    setSmartDriveAction(Dashboard.getInstance().getSmartDriveAction());

                } else if (robotState.getAlignOffset() == AlignOffset.MIDDLE_REEF) {
                    setSmartDriveAction(TargetAction.L1H);
                }
            } else if (RobotState.getFieldLocation() == FieldLocation.PROCESSOR) {
                setSmartDriveAction(TargetAction.AP);

            } else if (RobotState.getFieldLocation() == FieldLocation.BARGE) {

            } else if (RobotState.getFieldLocation() == FieldLocation.TRAVEL) {
                setSmartDriveAction(TargetAction.STOW);
            }
        }
    }

    private void setDriverAction(boolean driver) {
        if (!driver) {
            shouldSmartDrive = true;
        } else {
            if (getCurrentAction() == TargetAction.STOW && RobotState.getFieldLocation() == FieldLocation.TRAVEL) {
                shouldSmartDrive = true;
                driverAction = false;
            } else {
                shouldSmartDrive = false;
                driverAction = true;
            }
        }
    }

    public boolean willArmCrossDangerZone(double currentDeg, double goalDeg) {
        if (goalDeg > SuperstructureConstants.LEFT_LIMIT && currentDeg < SuperstructureConstants.RIGHT_LIMIT) {
            return true;
        } else if (goalDeg < SuperstructureConstants.RIGHT_LIMIT && currentDeg > SuperstructureConstants.LEFT_LIMIT) {
            return true;
        } else if (currentDeg > SuperstructureConstants.RIGHT_LIMIT
                && currentDeg < SuperstructureConstants.LEFT_LIMIT
                && (goalDeg > SuperstructureConstants.LEFT_LIMIT || goalDeg < SuperstructureConstants.RIGHT_LIMIT)) {
            return true;
        }

        return false;
    }

    public boolean atConfirmedPosition() {
        return atPosition(currentAction);
    }

    public boolean atPosition(TargetAction goalPosition) {
        return elevator.atPosition(goalPosition)
                && armPivot.atPosition(goalPosition)
                && intakePivot.atPosition(goalPosition);
    }

    public TargetAction getStowFromCurrent() {
        if (currentAction == TargetAction.ALGAE_NET) {
            return TargetAction.POST_ALGAE_STOW;
        } else if (currentAction == TargetAction.ALGAE_PROCESS) {
            return TargetAction.ALGAE_PROCESS;
        } else {
            return TargetAction.STOW;
        }
    }
}
