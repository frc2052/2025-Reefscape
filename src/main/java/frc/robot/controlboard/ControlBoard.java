package frc.robot.controlboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.controlboard.primary.GamepadPrimaryInput;
import frc.robot.controlboard.primary.IPrimaryControlBoard;
import frc.robot.controlboard.primary.JoystickPrimaryInput;
import frc.robot.controlboard.secondary.ControlPanelInput;
import frc.robot.controlboard.secondary.ISecondaryControlBoard;
import frc.robot.util.Ports;

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
  public Trigger resetGyro() {
    return primaryControlBoard.resetGyro();
  }

  @Override
  public Trigger reefAlignment() {
    return primaryControlBoard.reefAlignment();
  }

  @Override
  public Trigger intake() {
    return primaryControlBoard.intake();
  }

  @Override
  public Trigger shoot() {
    return primaryControlBoard.shoot();
  }

  /* Secondary */

  @Override
  public Trigger climb() {
    return secondaryControlBoard.climb();
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

  @Override
  public Trigger manualUp() {
    return secondaryControlBoard.manualUp();
  }

  @Override
  public Trigger manualDown() {
    return secondaryControlBoard.manualDown();
  }

  @Override
  public Trigger setElevatorPositionL1() {
    return secondaryControlBoard.setElevatorPositionL1();
  }

  @Override
  public Trigger setElevatorPositionL2() {
    return secondaryControlBoard.setElevatorPositionL2();
  }

  @Override
  public Trigger setElevatorPositionL3() {
    return secondaryControlBoard.setElevatorPositionL3();
  }

  @Override
  public Trigger setElevatorPositionL4() {
    return secondaryControlBoard.setElevatorPositionL4();
  }

  @Override
  public Trigger setElevatorPositionUpperAlgae() {
    return secondaryControlBoard.setElevatorPositionUpperAlgae();
  }

  @Override
  public Trigger setElevatorPositionLowerAlgae() {
    return secondaryControlBoard.setElevatorPositionLowerAlgae();
  }

  @Override
  public Trigger setElevatorPositionHandoff() {
    return secondaryControlBoard.setElevatorPositionHandoff();
  }

  @Override
  public Trigger setElevatorPositionTravel() {
    return secondaryControlBoard.setElevatorPositionTravel();
  }
}
