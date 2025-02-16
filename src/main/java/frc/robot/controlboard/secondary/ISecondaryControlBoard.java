package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ISecondaryControlBoard {
  Trigger homeElevator();

  Trigger actTrigger();

  Trigger manualUp();

  Trigger manualDown();

  Trigger climbUp();

  Trigger climbDown();

  Trigger reefAB();

  Trigger reefCD();

  Trigger reefEF();

  Trigger reefGH();

  Trigger reefIJ();

  Trigger reefKL();

  Trigger setGoalL1L();

  Trigger setGoalL1H();

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
