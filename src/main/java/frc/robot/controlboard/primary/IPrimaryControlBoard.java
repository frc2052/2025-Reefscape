package frc.robot.controlboard.primary;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IPrimaryControlBoard {
  double getThrottle();

  double getStrafe();

  double getRotation();

  EventLoop povLooper = CommandScheduler.getInstance().getDefaultButtonLoop();

  Trigger povUp();

  Trigger povUpRight();

  Trigger povRight();

  Trigger povDownRight();

  Trigger povDown();

  Trigger povDownLeft();

  Trigger povLeft();

  Trigger povUpLeft();

  Trigger povRotLeft();

  Trigger povRotRight();

  Trigger resetGyro();

  Trigger outtakeCoral();

  Trigger intakeCoral();

  Trigger intakeAlgae();

  Trigger shootAlgae();

  Trigger reefAlignment();

  Trigger sysIDQuasiForward();

  Trigger sysIDQuasiReverse();

  Trigger sysIDDynamicForward();

  Trigger sysIDDynamicReverse();
}
