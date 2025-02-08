package frc.robot.subsystems;

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
    isChangingState = false;
  }

  /** Public method to provide access to the instance. */
  public static SuperstructureSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SuperstructureSubsystem();
    }
    return INSTANCE;
  }

  @Override
  public void periodic() {
    TargetAction target = position.getTargetAction();

    if (target != previousAction) {
      isChangingState = true;
    }

    if (isChangingState) {

      if (elevator.atPosition(target)) {
        coralArm.setArmPosition(target);
        algaeArm.setGoalPosition(target);
      } else {
        if (coralArm.isAtPosition(5, TargetAction.TR.getCoralArmAngle()) && coralArm.isAtPosition(5, TargetAction.TR.getAlgaeArmPosition())) {
          elevator.setPositionMotionMagic(target);
        } else {
          elevator.stopElevator();
          coralArm.setArmPosition(TargetAction.TR);
          algaeArm.setGoalPosition(TargetAction.TR);
        }
      }

      if (elevator.atPosition(target) && coralArm.isAtPosition(5, target.getCoralArmAngle()) && algaeArm.isAtPosition(5, target.getAlgaeArmPosition())) {
        isChangingState = false;
        System.out.println("Arrived at Target State");
      }

    }

    previousAction = target;
  }
}
