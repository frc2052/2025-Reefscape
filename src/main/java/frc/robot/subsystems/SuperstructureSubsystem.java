package frc.robot.subsystems;

import com.team2052.lib.util.SecondaryImageManager;
import com.team2052.lib.util.SecondaryImageManager.SecondaryImage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldLocation;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.PositionSuperstructure;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;
import org.littletonrobotics.junction.Logger;

public class SuperstructureSubsystem extends SubsystemBase {

  private static SuperstructureSubsystem INSTANCE;

  private AlgaeSubsystem algaeArm = AlgaeSubsystem.getInstance();
  private ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private CoralArmSubsystem coralArm = CoralArmSubsystem.getInstance();
  private PositionSuperstructure position = PositionSuperstructure.getInstance();

  private TargetAction previousAction;
  private boolean isChangingState;
  private boolean travelling;

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

  @Override
  public void periodic() {
    TargetAction target = position.getTargetAction();
    Logger.recordOutput("Target Superstructure Changing State", isChangingState);

    if (target != previousAction) {
      Logger.recordOutput("Target Superstructure State Has Changed", true);
      pushChangedValueToShuffleboard(target);
      isChangingState = true;
    } else {
      Logger.recordOutput("Target Superstructure State Has Changed", false);
    }

    if (isChangingState && ControlBoard.getInstance().actTrigger().getAsBoolean()) {
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
    // if (isChangingState) {
    //   if (elevator.atPosition(2.5, target)) {
    //     coralArm.setArmPosition(target);
    //     algaeArm.setGoalPosition(target);
    //     System.out.println("ONE");
    //   } else {
    //     if (coralArm.isAtPosition(25, TargetAction.TR.getCoralArmAngle())
    //         && algaeArm.isAtPosition(10, TargetAction.TR.getAlgaeArmPosition())) {
    //       elevator.setPositionMotionMagic(target);
    //       System.out.println("TWO");
    //     } else {
    //       elevator.stopElevator();
    //       coralArm.setArmPosition(TargetAction.TR);
    //       algaeArm.setGoalPosition(TargetAction.TR);
    //       System.out.println("THREE");
    //     }
    //   }

    //   if (elevator.atPosition(target)
    //       && coralArm.isAtPosition(5, target.getCoralArmAngle())
    //       && algaeArm.isAtPosition(5, target.getAlgaeArmPosition())) {
    //     isChangingState = false;
    //     Logger.recordOutput("Arrived at Target State", true);
    //   } else {
    //     Logger.recoxxxxxrdOutput("Arrived at Target State", false);
    //   }
    // }

    previousAction = target;
    setTargetAction();
  }

  private void
      setTargetAction() { // todo: these following lines of code requires more fine tuning, which we
    // can do if we pursue with this aproach.
    if (RobotState.getFieldLocation() == FieldLocation.HP) {
      PositionSuperstructure.getInstance().setTargetAction(TargetAction.HP);
    } else if (RobotState.getFieldLocation() == FieldLocation.REEF) {
      PositionSuperstructure.getInstance().setTargetAction(TargetAction.L3);
    } else if (RobotState.getFieldLocation() == FieldLocation.PROCESSOR) {
      PositionSuperstructure.getInstance().setTargetAction(TargetAction.HM);
    } else if (RobotState.getFieldLocation() == FieldLocation.BARGE) {
      if (AlgaeSubsystem.getHasAlgae()) {
        PositionSuperstructure.getInstance().setTargetAction(TargetAction.AS);
      } else {
        PositionSuperstructure.getInstance().setTargetAction(TargetAction.HM);
      }
    } else {
      PositionSuperstructure.getInstance().setTargetAction(TargetAction.HM);
    }
  }
}
