package frc.robot.subsystems;

import com.team2052.lib.util.SecondaryImageManager;
import com.team2052.lib.util.SecondaryImageManager.SecondaryImage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.controlboard.PositionSuperstructure;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;

public class SuperstructureSubsystem extends SubsystemBase {

  private static SuperstructureSubsystem INSTANCE;

  private AlgaeSubsystem algaeArm = AlgaeSubsystem.getInstance();
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private ArmSubsystem coralArm = ArmSubsystem.getInstance();
  private PositionSuperstructure position = PositionSuperstructure.getInstance();

  private TargetAction previousAction;
  private boolean isChangingState;

  /** Private constructor to prevent instantiation. */
  private SuperstructureSubsystem() {
    previousAction = position.getTargetAction();
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
      case L1:
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
        SecondaryImageManager.setCurrentImage(SecondaryImage.A1);
        break;
      case LA:
        SecondaryImageManager.setCurrentImage(SecondaryImage.A2);
        break;
      default:
        SecondaryImageManager.setCurrentImage(SecondaryImage.NONE);
        break;
    }
  }

  @Override
  public void periodic() {
    TargetAction target = position.getTargetAction();

    if (target != previousAction) {
      System.out.println("Target State Has Changed");
      pushChangedValueToShuffleboard(target);
      isChangingState = true;
    }

    if (isChangingState) {

      if (elevator.atPosition(target)) {
        coralArm.setArmPosition(target);
        algaeArm.setGoalPosition(target);
      } else {
        if (coralArm.isAtPosition(5, TargetAction.TR.getCoralArmAngle())
            && coralArm.isAtPosition(5, TargetAction.TR.getAlgaeArmPosition())) {
          elevator.setPositionMotionMagic(target);
        } else {
          elevator.stopElevator();
          coralArm.setArmPosition(TargetAction.TR);
          algaeArm.setGoalPosition(TargetAction.TR);
        }
      }

      if (elevator.atPosition(target)
          && coralArm.isAtPosition(5, target.getCoralArmAngle())
          && algaeArm.isAtPosition(5, target.getAlgaeArmPosition())) {
        isChangingState = false;
        System.out.println("Arrived at Target State");
      }
    }

    previousAction = target;
  }
}
