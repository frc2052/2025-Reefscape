package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import com.team2052.lib.util.SecondaryImageManager;
import com.team2052.lib.util.SecondaryImageManager.SecondaryImage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldLocation;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSubsystem extends SubsystemBase {

    private static SuperstructureSubsystem INSTANCE;

    private RobotState robotState = RobotState.getInstance();
    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private ArmPivotSubsystem armPivot = ArmPivotSubsystem.getInstance();
    private IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();

    private TargetAction selectedTargetAction = TargetAction.TR;
    private TargetAction currentAction = TargetAction.TR;

    private TargetAction previousAction;
    private boolean isChangingState;

    private boolean cancelHome = false;
    private boolean driverAction;
    private boolean shouldSmartDrive;
    private boolean algaeScoreDownNeeded = false;
    private boolean movingFromIntake = false;

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

    public boolean isAtTargetState() {
        return isChangingState;
    }

    public TargetAction getLatestAction() {
        return previousAction;
    }

    private void pushChangedValueToShuffleboard(TargetAction action) {
        switch (action) {
            case L1H:
                SecondaryImageManager.setCurrentImage(SecondaryImage.L1);
                break;
            case L2:
                SecondaryImageManager.setCurrentImage(SecondaryImage.L2);
                break;
            case L3:
                SecondaryImageManager.setCurrentImage(SecondaryImage.L3);
                break;
            case L4:
                SecondaryImageManager.setCurrentImage(SecondaryImage.L4);
                break;
            case UA:
                SecondaryImageManager.setCurrentImage(SecondaryImage.A2);
                break;
            case LA:
                SecondaryImageManager.setCurrentImage(SecondaryImage.A1);
                break;
            default:
                SecondaryImageManager.setCurrentImage(SecondaryImage.NONE);
                break;
        }
    }

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
        TargetAction target = getCurrentAction();
        Logger.recordOutput("Superstructure/Current = Selected", target == getSelectedTargetAction());
        Logger.recordOutput("Target Superstructure Changing State", isChangingState);

        if (target != previousAction) {
            Logger.recordOutput("Target Superstructure State Has Changed", true);
            isChangingState = true;
            if (target != TargetAction.HM) {
                cancelHome = true;
            }
        } else {
            Logger.recordOutput("Target Superstructure State Has Changed", false);
        }

        if (target == TargetAction.AS) {
            algaeScoreDownNeeded = true;
        }

        if (!movingFromIntake
                && RobotState.getInstance().getHasCoral()
                && (target == TargetAction.INTAKE)
                && armPivot.atPosition(target)) {
            System.out.println("GOT CORAL********************");
            movingFromIntake = true;
            setCurrentAction(TargetAction.STOW);
        }

        if (armPivot.atPosition(TargetAction.STOW)) {
            movingFromIntake = false;
        }

        if (isChangingState) {
            if (cancelHome) {
                elevator.setWantHome(false);
                cancelHome = false;
            } else if (target == TargetAction.HM && !elevator.isHoming()) {
                elevator.setWantHome(true);
                System.out.println("HOMING");
                return;
            }

            boolean armCrossingDanger = willArmCrossDangerZone(
                    armPivot.getArmAngle().in(Degrees),
                    target.getArmPivotAngle().in(Degrees));

            if (target.getElevatorPositionRotations() < SuperstructureConstants.MIN_SAFE_ROTATION
                    && elevator.getPosition() < SuperstructureConstants.MIN_SAFE_ROTATION
                    && armCrossingDanger) {
                intakePivot.setPosition(target);

                if (!armPivot.isAtPosition(5, target.getArmPivotAngle())) {
                    elevator.setPositionMotionMagic(TargetAction.SAFE_ARM_HEIGHT);
                }

                if (elevator.atPosition(1.0, TargetAction.SAFE_ARM_HEIGHT)) {
                    armPivot.setArmPosition(target);
                    if (armPivot.isAtPosition(5, target.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(target);
                    }
                }
            } else if (target.getElevatorPositionRotations() < elevator.getPosition()) {
                if (algaeScoreDownNeeded) {
                    armPivot.setArmPosition(target);
                    intakePivot.setPosition(target);

                    if (armPivot.isAtPosition(3, target.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(target);
                        algaeScoreDownNeeded = false;
                    }
                }
                if (target.getElevatorPositionRotations() > SuperstructureConstants.MIN_SAFE_ROTATION
                        || elevator.getPosition() > SuperstructureConstants.MIN_MOVE_ROTATION) {
                    elevator.setPositionMotionMagic(target);
                    armPivot.setArmPosition(target);
                    intakePivot.setPosition(target);
                } else {
                    armPivot.setArmPosition(target);
                    intakePivot.setPosition(target);

                    if (armPivot.isAtPosition(10, target.getArmPivotAngle())) {
                        elevator.setPositionMotionMagic(target);
                    }
                }
            } else if (target.getElevatorPositionRotations() > elevator.getPosition()) {
                elevator.setPositionMotionMagic(target);
                intakePivot.setPosition(target);

                if (elevator.atPosition(5, target)) {
                    armPivot.setArmPosition(target);
                }
            }

            if (elevator.atPosition(target) && armPivot.isAtPosition(5, target.getArmPivotAngle())) {
                isChangingState = false;
                Logger.recordOutput("Arrived at Target State", true);
            } else {
                Logger.recordOutput("Arrived at Target State", false);
            }
        }
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
        if (shouldSmartDrive) {
            if (RobotState.getFieldLocation() == FieldLocation.REEF
                    && IntakeRollerSubsystem.getInstance().getHasCoral()) {
                System.out.println("inside the zone");
                if (robotState.getAlignOffset() == AlignOffset.LEFT_BRANCH
                        || robotState.getAlignOffset() == AlignOffset.RIGHT_BRANCH) {
                    setSmartDriveAction(TargetAction.L3);

                } else if (robotState.getAlignOffset() == AlignOffset.MIDDLE_REEF) {
                    setSmartDriveAction(TargetAction.L1H);
                }
            } else if (RobotState.getFieldLocation() == FieldLocation.PROCESSOR) {
                setSmartDriveAction(TargetAction.AP);

            } else if (RobotState.getFieldLocation() == FieldLocation.BARGE) {

            } else if (RobotState.getFieldLocation() == FieldLocation.TRAVEL) {
                // setSmartDriveAction(TargetAction.STOW);
            }
        }
    }

    private void setDriverAction(boolean driver) {
        if (!driver) {
            shouldSmartDrive = true;
        } else {
            if (getCurrentAction() == TargetAction.STOW && RobotState.getFieldLocation() == FieldLocation.TRAVEL) {
                shouldSmartDrive = true;
            } else {
                shouldSmartDrive = false;
            }
        }
    }

    public boolean willArmCrossDangerZone(double currentDeg, double goalDeg) {
        if (goalDeg > SuperstructureConstants.LEFT_LIMIT && currentDeg < SuperstructureConstants.RIGHT_LIMIT) {
            return true;
        } else if (goalDeg < SuperstructureConstants.RIGHT_LIMIT && currentDeg > SuperstructureConstants.LEFT_LIMIT) {
            return true;
        } else if (currentDeg > SuperstructureConstants.RIGHT_LIMIT
                && currentDeg < SuperstructureConstants.LEFT_LIMIT) {
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
        if (currentAction == TargetAction.AS) {
            return TargetAction.POST_ALGAE_STOW;
        } else if (currentAction == TargetAction.AP) {
            return TargetAction.AP;
        } else {
            return TargetAction.STOW;
        }
    }
}
