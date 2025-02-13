package frc.robot.controlboard.primary;

import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.Ports;

public class JoystickPrimaryInput implements IPrimaryControlBoard {
  private final Joystick translateStick;
  private final Joystick rotateStick;

  private static JoystickPrimaryInput INSTANCE;

  public static JoystickPrimaryInput getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new JoystickPrimaryInput();
    }
    return INSTANCE;
  }

  private JoystickPrimaryInput() {
    translateStick = new Joystick(Ports.TRANSLATION_JOYSTICK_PORT);
    rotateStick = new Joystick(Ports.ROTATION_JOYSTICK_PORT);
  }

  @Override
  public double getThrottle() {
    return MathHelpers.deadband(-translateStick.getRawAxis(1), DriverConstants.JOYSTICK_DEADBAND);
  }

  @Override
  public double getStrafe() {
    return MathHelpers.deadband(-translateStick.getRawAxis(0), DriverConstants.JOYSTICK_DEADBAND);
  }

  @Override
  public double getRotation() {
    return MathHelpers.deadband(-rotateStick.getRawAxis(0), DriverConstants.JOYSTICK_DEADBAND);
  }

  @Override
  public Trigger povUp() {
    return new Trigger(translateStick.povUp(povLooper));
  }

  @Override
  public Trigger povUpRight() {
    return new Trigger(translateStick.povUpRight(povLooper));
  }

  @Override
  public Trigger povRight() {
    return new Trigger(translateStick.povRight(povLooper));
  }

  @Override
  public Trigger povDownRight() {
    return new Trigger(translateStick.povDownRight(povLooper));
  }

  @Override
  public Trigger povDown() {
    return new Trigger(translateStick.povDown(povLooper));
  }

  @Override
  public Trigger povDownLeft() {
    return new Trigger(translateStick.povDownLeft(povLooper));
  }

  @Override
  public Trigger povLeft() {
    return new Trigger(translateStick.povLeft(povLooper));
  }

  @Override
  public Trigger povUpLeft() {
    return new Trigger(translateStick.povUpLeft(povLooper));
  }

  @Override
  public Trigger povRotLeft() {
    return new Trigger(rotateStick.povLeft(povLooper));
  }

  @Override
  public Trigger povRotRight() {
    return new Trigger(rotateStick.povRight(povLooper));
  }

  @Override
  public Trigger resetGyro() {
    return new JoystickButton(translateStick, 8);
  }

  @Override
  public Trigger outtake() {
    return new JoystickButton(translateStick, 2);
  }

  @Override
  public Trigger intake() {
    return new JoystickButton(translateStick, 1);
  }

  @Override
  public Trigger shoot() {
    return new JoystickButton(rotateStick, 1);
  }

  @Override
  public Trigger reefAlignment() {
    return new JoystickButton(rotateStick, 3);
  }

  @Override
  public Trigger sysIDQuasiForward() {
    return new JoystickButton(translateStick, 7);
  }

  @Override
  public Trigger sysIDQuasiReverse() {
    return new JoystickButton(translateStick, 6);
  }

  @Override
  public Trigger sysIDDynamicForward() {
    return new JoystickButton(translateStick, 5);
  }

  @Override
  public Trigger sysIDDynamicReverse() {
    return new JoystickButton(translateStick, 10);
  }
}
