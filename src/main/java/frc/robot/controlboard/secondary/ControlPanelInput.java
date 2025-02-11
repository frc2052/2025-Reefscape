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
  public Trigger homeElevator() {
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
  public Trigger reefAB() {
    return new JoystickButton(controlPanel, 3);
  }

  @Override
  public Trigger reefCD() {
    return new JoystickButton(controlPanel, 4);
  }

  @Override
  public Trigger reefEF() {
    return new JoystickButton(controlPanel, 10);
  }

  @Override
  public Trigger reefGH() {
    return new JoystickButton(controlPanel, 11);
  }

  @Override
  public Trigger reefIJ() {
    return new JoystickButton(controlPanel, 12);
  }

  @Override
  public Trigger reefKL() {
    return new JoystickButton(controlPanel, 5);
  }

  @Override
  public Trigger setGoalL1() {
    return new JoystickButton(controlPanel, 11);
  }

  @Override
  public Trigger setGoalL2() {
    return new JoystickButton(controlPanel, 12);
  }

  @Override
  public Trigger setGoalL3() {
    return new JoystickButton(controlPanel, 5);
  }

  @Override
  public Trigger setGoalL4() {
    return new JoystickButton(controlPanel, 7);
  }

  @Override
  public Trigger setGoalUpperAlgae() {
    return new JoystickButton(controlPanel, 8);
  }

  @Override
  public Trigger setGoalLowerAlgae() {
    return new JoystickButton(controlPanel, 9);
  }

  @Override
  public Trigger setSubReefLeft() {
    return new JoystickButton(controlPanel, 99);
  }

  @Override
  public Trigger setSubReefCenter() {
    return new JoystickButton(controlPanel, 99);
  }

  @Override
  public Trigger setSubReefRight() {
    return new JoystickButton(controlPanel, 99);
  }

  @Override
  public Trigger setGoalCoralStation() {
    return new JoystickButton(controlPanel, 99);
  }

  @Override
  public Trigger setGoalAlgaeScoring() {
    return new JoystickButton(controlPanel, 99);
  }

  @Override
  public Trigger setGoalTravel() {
    return new JoystickButton(controlPanel, 99);
  }
}
