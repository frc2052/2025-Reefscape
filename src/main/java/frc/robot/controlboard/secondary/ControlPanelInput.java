package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.io.Ports;

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
        return new JoystickButton(controlPanel, 10);
    }

    @Override
    public Trigger actTrigger() {
        return new JoystickButton(controlPanel, 2);
    }

    @Override
    public Trigger algaeScoreAngle() {
        return new JoystickButton(controlPanel, 3);
    }

    @Override
    public Trigger algaeLowAngle() {
        return new JoystickButton(controlPanel, 4);
    }

    @Override
    public Trigger climbUp() {
        return new Trigger(() -> controlPanel.getY() > 0.5);
    }

    @Override
    public Trigger climbDown() {
        return new Trigger(() -> controlPanel.getY() < -0.5);
    }

    @Override
    public Trigger setGoalCL() {
        return new JoystickButton(controlPanel, 9);
    }

    @Override
    public Trigger setGoalL1H() {
        return new JoystickButton(controlPanel, 12);
    }

    @Override
    public Trigger setGoalL2() {
        return new JoystickButton(controlPanel, 5);
    }

    @Override
    public Trigger setGoalL3() {
        return new JoystickButton(controlPanel, 7);
    }

    @Override
    public Trigger setGoalL4() {
        return new JoystickButton(controlPanel, 8);
    }

    @Override
    public Trigger setGoalUpperAlgae() {
        return new JoystickButton(controlPanel, 1);
    }

    @Override
    public Trigger setGoalLowerAlgae() {
        return new JoystickButton(controlPanel, 6);
    }

    @Override
    public Trigger setSubReefLeft() {
        return new Trigger(() -> controlPanel.getX() > 0.5);
    }

    @Override
    public Trigger setSubReefRight() {
        return new Trigger(() -> controlPanel.getX() < -0.5);
    }

    @Override
    public Trigger setGoalCoralStation() {
        return new JoystickButton(controlPanel, 11);
    }
}
