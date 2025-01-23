package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ISecondaryControlBoard {
  Trigger climb();

  Trigger manualUp();

  Trigger manualDown();

  Trigger setElevatorPositionL1();

  Trigger setElevatorPositionL2();

  Trigger setElevatorPositionL3();

  Trigger setElevatorPositionL4();

  Trigger setElevatorPositionUpperAlgae();

  Trigger setElevatorPositionLowerAlgae();

  Trigger setElevatorPositionHandoff();

  Trigger setElevatorPositionTravel();
}
