package frc.robot.subsystems.superstructure;

import com.team2052.lib.util.SecondaryImageManager;
import com.team2052.lib.util.SecondaryImageManager.SecondaryImage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSubsystem extends SubsystemBase {

    private static SuperstructureSubsystem INSTANCE;

    private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private ArmPivotSubsystem armPivot = ArmPivotSubsystem.getInstance();
    private IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();

    private TargetAction selectedTargetAction = TargetAction.TR;
    private TargetAction currentAction = TargetAction.TR;

    private TargetAction previousAction;
    private boolean isChangingState;

    private boolean cancelHome = false;

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

    public void setCurrentAction(TargetAction target) {
        currentAction = target;
    }

    public Command confirm() {
        return new InstantCommand(() -> confirmSelectedAction());
    }

    public void confirmSelectedAction() {
        currentAction = selectedTargetAction;
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

        if (isChangingState) {
            if (cancelHome) {
                elevator.setWantHome(false);
                cancelHome = false;
            } else if (target == TargetAction.HM) {
                elevator.setWantHome(true);
                System.out.println("HOMING");
                return;
            }

            if (target.getElevatorPositionRotations() > elevator.getPosition()) { // going up
                elevator.setPositionMotionMagic(target);
                System.out.println("STAGE 1");
                if (elevator.getPosition() > SuperstructureConstants.UPWARDS_MIN_ELEVATOR) {
                    armPivot.setArmPosition(target);
                    intakePivot.setPosition(target);
                    System.out.println("STAGE 2");
                }
            } else if (target.getElevatorPositionRotations() < elevator.getPosition()) { // going down
                if (target.getElevatorPositionRotations() > SuperstructureConstants.UPWARDS_MIN_ELEVATOR) {
                    armPivot.setArmPosition(target);
                    intakePivot.setPosition(target);
                    elevator.setPositionMotionMagic(target);
                } else {
                    armPivot.setArmPosition(target);
                    intakePivot.setPosition(target);
                    System.out.println("STAGE 3");

                    if (armPivot.isAtPosition(
                            5,
                            target.getArmPivotAngle())) { // Degrees.of(SuperstructureConstants.DOWNWARDS_ARM_ANGLE))) {
                        elevator.setPositionMotionMagic(target);
                        System.out.println("STAGE 4");
                    }
                }
            }

            if (elevator.atPosition(target) && armPivot.isAtPosition(5, target.getArmPivotAngle())) {
                // && algaePivot.isAtPosition(5, target.getAlgaeArmPivotPosition())) {
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
    }
}
