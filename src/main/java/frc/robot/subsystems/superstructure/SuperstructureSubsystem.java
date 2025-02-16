package frc.robot.subsystems.superstructure;

import com.team2052.lib.util.SecondaryImageManager;
import com.team2052.lib.util.SecondaryImageManager.SecondaryImage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSubsystem extends SubsystemBase {

  private static SuperstructureSubsystem INSTANCE;

  private AlgaeSubsystem algaeArm = AlgaeSubsystem.getInstance();
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private CoralArmSubsystem coralArm = CoralArmSubsystem.getInstance();

  private TargetAction selectedTargetAction = TargetAction.TR;
  private TargetAction currentAction = TargetAction.TR;

  private TargetAction previousAction;
  private boolean isChangingState;

  /** Private constructor to prevent instantiation. */
  private SuperstructureSubsystem() {
    previousAction = getSelectedTargetAction();
    pushChangedValueToShuffleboard(previousAction);
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

  public void setSelectedTargetAction(TargetAction target, boolean confirm) {
    selectedTargetAction = target;
    if (confirm) {
      confirmSelectedAction();
    }
    revealCombination();
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
      pushChangedValueToShuffleboard(target);
      isChangingState = true;
    } else {
      Logger.recordOutput("Target Superstructure State Has Changed", false);
    }

    if (isChangingState) {
      if (target == TargetAction.HM && elevator.shouldHome()) {
        elevator.homeElevator().schedule();
      } else {
        elevator.setPositionMotionMagic(target);
      }
      coralArm.setArmPosition(target);
      algaeArm.setGoalPosition(target);

      if (elevator.atPosition(target)
          && coralArm.isAtPosition(5, target.getCoralArmAngle())
          && algaeArm.isAtPosition(5, target.getAlgaeArmPosition())) {
        isChangingState = false;
        Logger.recordOutput("Arrived at Target State", true);
      } else {
        Logger.recordOutput("Arrived at Target State", false);
      }
    }

    previousAction = target;
  }
}
