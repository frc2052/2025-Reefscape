package frc.robot.controlboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.controlboard.primary.GamepadPrimaryInput;
import frc.robot.controlboard.primary.IPrimaryControlBoard;
import frc.robot.controlboard.primary.JoystickPrimaryInput;
import frc.robot.controlboard.secondary.ControlPanelInput;
import frc.robot.controlboard.secondary.ISecondaryControlBoard;
import frc.robot.util.io.Ports;

public class ControlBoard implements IPrimaryControlBoard, ISecondaryControlBoard {
  private final IPrimaryControlBoard primaryControlBoard;
  private final ISecondaryControlBoard secondaryControlBoard;

  private static ControlBoard INSTANCE;

  public static ControlBoard getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ControlBoard();
    }
    return INSTANCE;
  }

  private ControlBoard() {
    boolean useDriveGamepad =
        DriverStation.getJoystickIsXbox(Ports.GAMEPAD_PORT) || DriverConstants.FORCE_GAMEPAD;
    primaryControlBoard =
        useDriveGamepad ? GamepadPrimaryInput.getInstance() : JoystickPrimaryInput.getInstance();
    secondaryControlBoard = ControlPanelInput.getInstance();

    if (!useDriveGamepad) {
      IPrimaryControlBoard.povLooper.poll();
    }
  }

  /* Primary */

  @Override
  public double getThrottle() {
    return primaryControlBoard.getThrottle();
  }

  @Override
  public double getStrafe() {
    return primaryControlBoard.getStrafe();
  }

  @Override
  public double getRotation() {
    return primaryControlBoard.getRotation();
  }

  @Override
  public Trigger povUp() {
    return primaryControlBoard.povUp();
  }

  @Override
  public Trigger povUpRight() {
    return primaryControlBoard.povUpRight();
  }

  @Override
  public Trigger povRight() {
    return primaryControlBoard.povRight();
  }

  @Override
  public Trigger povDownRight() {
    return primaryControlBoard.povDownRight();
  }

  @Override
  public Trigger povDown() {
    return primaryControlBoard.povDown();
  }

  @Override
  public Trigger povDownLeft() {
    return primaryControlBoard.povDownLeft();
  }

  @Override
  public Trigger povLeft() {
    return primaryControlBoard.povLeft();
  }

  @Override
  public Trigger povUpLeft() {
    return primaryControlBoard.povUpLeft();
  }

  @Override
  public Trigger povRotLeft() {
    return primaryControlBoard.povRotLeft();
  }

  @Override
  public Trigger povRotRight() {
    return primaryControlBoard.povRotRight();
  }

  @Override
  public Trigger algaeManualUp() {
    return primaryControlBoard.algaeManualUp();
  }

  @Override
  public Trigger algaeManualDown() {
    return primaryControlBoard.algaeManualDown();
  }

  @Override
  public Trigger resetGyro() {
    return primaryControlBoard.resetGyro();
  }

  @Override
  public Trigger outtake() {
    return primaryControlBoard.outtake();
  }

  @Override
  public Trigger intake() {
    return primaryControlBoard.intake();
  }

  @Override
  public Trigger outtakeAlgae() {
    return primaryControlBoard.outtakeAlgae();
  }

  @Override
  public Trigger intakeAlgae() {
    return primaryControlBoard.intakeAlgae();
  }

  @Override
  public Trigger alignWithElement() {
    return primaryControlBoard.alignWithElement();
  }

  @Override
  public Trigger sysIDQuasiForward() {
    return primaryControlBoard.sysIDQuasiForward();
  }

  @Override
  public Trigger sysIDQuasiReverse() {
    return primaryControlBoard.sysIDQuasiReverse();
  }

  @Override
  public Trigger sysIDDynamicForward() {
    return primaryControlBoard.sysIDDynamicForward();
  }

  @Override
  public Trigger sysIDDynamicReverse() {
    return primaryControlBoard.sysIDDynamicReverse();
  }

  /* Secondary */

  @Override
  public Trigger homeElevator() {
    return secondaryControlBoard.homeElevator();
  }

  @Override
  public Trigger actTrigger() {
    return secondaryControlBoard.actTrigger();
  }

  @Override
  public Trigger algaeScoreAngle() {
    return secondaryControlBoard.algaeScoreAngle();
  }

  @Override
  public Trigger algaeLowAngle() {
    return secondaryControlBoard.algaeLowAngle();
  }

  @Override
  public Trigger climbUp() {
    return secondaryControlBoard.climbUp();
  }

  @Override
  public Trigger climbDown() {
    return secondaryControlBoard.climbDown();
  }

  @Override
  public Trigger setGoalCL() {
    return secondaryControlBoard.setGoalCL();
  }

  @Override
  public Trigger setGoalL1H() {
    return secondaryControlBoard.setGoalL1H();
  }

  @Override
  public Trigger setGoalL2() {
    return secondaryControlBoard.setGoalL2();
  }

  @Override
  public Trigger setGoalL3() {
    return secondaryControlBoard.setGoalL3();
  }

  @Override
  public Trigger setGoalL4() {
    return secondaryControlBoard.setGoalL4();
  }

  @Override
  public Trigger setGoalUpperAlgae() {
    return secondaryControlBoard.setGoalUpperAlgae();
  }

  @Override
  public Trigger setGoalLowerAlgae() {
    return secondaryControlBoard.setGoalLowerAlgae();
  }

  @Override
  public Trigger setSubReefLeft() {
    return secondaryControlBoard.setSubReefLeft();
  }

  @Override
  public Trigger setSubReefRight() {
    return secondaryControlBoard.setSubReefRight();
  }

  @Override
  public Trigger setGoalCoralStation() {
    return secondaryControlBoard.setGoalCoralStation();
  }
}
