package frc.robot.controlboard.secondary;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ISecondaryControlBoard {
    Trigger homeElevator();

    Trigger actTrigger();

    Trigger algaeScoreAngle();

    Trigger algaeLowAngle();

    Trigger climbUp();

    Trigger climbDown();

    Trigger setGoalCL();

    Trigger setGoalL1H();

    Trigger setGoalL2();

    Trigger setGoalL3();

    Trigger setGoalL4();

    Trigger setGoalUpperAlgae();

    Trigger setGoalLowerAlgae();

    Trigger setGoalCoralStation();

    Trigger unJam();

    Trigger loadingStation();
}
