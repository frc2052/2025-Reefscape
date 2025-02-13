package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ISecondaryControlBoard {
  Trigger homeElevator();

  Trigger manualUp();

  Trigger manualDown();

  Trigger reefAB();

  Trigger reefCD();

  Trigger reefEF();

  Trigger reefGH();

  Trigger reefIJ();

  Trigger reefKL();

  Trigger setHandoff();

  Trigger setGoalL1();

  Trigger setGoalL2();

  Trigger setGoalL3();

  Trigger setGoalL4();

  Trigger setGoalUpperAlgae();

  Trigger setGoalLowerAlgae();

  Trigger setGoalCoralStation();

  Trigger setGoalAlgaeScoring();

  Trigger setGoalTravel();

  Trigger setSubReefLeft();

  Trigger setSubReefCenter();

  Trigger setSubReefRight();
}
