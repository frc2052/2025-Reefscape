package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Ports;

public class ControlPanelInput implements ISecondaryControlBoard {
  private final Joystick controlPanel;

  private static ControlPanelInput INSTANCE;

  public static ControlPanelInput getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ControlPanelInput();
    }
    return INSTANCE;
  }

  private ControlPanelInput() {
    controlPanel = new Joystick(Ports.CONTROL_PANEL_PORT);
  }

  @Override
  public Trigger zeroElevator() {
    return new JoystickButton(controlPanel, 2);
  }

  @Override
  public Trigger manualUp() {
    return new JoystickButton(controlPanel, 1);
  }

  @Override
  public Trigger manualDown() {
    return new JoystickButton(controlPanel, 6);
  }

  @Override
  public Trigger setElevatorPositionL1() {
    return new JoystickButton(controlPanel, 11);
  }

  @Override
  public Trigger setElevatorPositionL2() {
    return new JoystickButton(controlPanel, 12);
  }

  @Override
  public Trigger setElevatorPositionL3() {
    return new JoystickButton(controlPanel, 5);
  }

  @Override
  public Trigger setElevatorPositionL4() {
    return new JoystickButton(controlPanel, 7);
  }

  @Override
  public Trigger setElevatorPositionUpperAlgae() {
    return new JoystickButton(controlPanel, 8);
  }

  @Override
  public Trigger setElevatorPositionLowerAlgae() {
    return new Trigger(() -> false);
    // return new JoystickButton(controlPanel, 9);
  }

  @Override
  public Trigger setElevatorPositionHandoff() {
    return new Trigger(() -> false);
    // return new JoystickButton(controlPanel, 10);
  }

  @Override
  public Trigger setElevatorPositionTravel() {
    return new Trigger(() -> controlPanel.getX() < -0.5);
  }
}
