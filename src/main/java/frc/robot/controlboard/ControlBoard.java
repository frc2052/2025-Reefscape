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
  public Trigger povRotLeft() {
    return primaryControlBoard.povRotLeft();
  }

  @Override
  public Trigger povRotRight() {
    return primaryControlBoard.povRotRight();
  }

  @Override
  public Trigger resetGyro() {
    return primaryControlBoard.resetGyro();
  }

  @Override
  public Trigger distanceToTag() {
    return primaryControlBoard.distanceToTag();
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
  public Trigger manualUp() {
    return secondaryControlBoard.manualUp();
  }

  @Override
  public Trigger manualDown() {
    return secondaryControlBoard.manualDown();
  }

  @Override
  public Trigger reefAB() {
    return secondaryControlBoard.reefAB();
  }

  @Override
  public Trigger reefCD() {
    return secondaryControlBoard.reefCD();
  }

  @Override
  public Trigger reefEF() {
    return secondaryControlBoard.reefEF();
  }

  @Override
  public Trigger reefGH() {
    return secondaryControlBoard.reefGH();
  }

  @Override
  public Trigger reefIJ() {
    return secondaryControlBoard.reefIJ();
  }

  @Override
  public Trigger reefKL() {
    return secondaryControlBoard.reefKL();
  }

  @Override
  public Trigger setGoalL1() {
    return secondaryControlBoard.setGoalL1();
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
}
