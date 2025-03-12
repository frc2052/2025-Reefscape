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

    Trigger algaeManualUp();

    Trigger algaeManualDown();

    Trigger resetGyro();

    Trigger alignWithReefLeft();

    Trigger alignWithReefRight();

    Trigger outtake();

    Trigger intake();

    Trigger shootAlgae();

    Trigger intakeAlgae();

    Trigger sysIDQuasiForward();

    Trigger sysIDQuasiReverse();

    Trigger sysIDDynamicForward();

    Trigger sysIDDynamicReverse();
}
