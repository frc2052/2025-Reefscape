// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotState;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;
import frc.robot.util.Ports;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final RobotState robotState = RobotState.getInstance();
  private final TalonFX pivotMotor;
  private Angle goalPosition;

  private static ArmSubsystem INSTANCE;

  public static ArmSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ArmSubsystem();
    }
    return INSTANCE;
  }

  private ArmSubsystem() {
    goalPosition = TargetAction.HP.coralArmAngle;

    pivotMotor = new TalonFX(Ports.ARM_TALONFX_ID);

    pivotMotor.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);
  }

  // private void setArmAngle(Angle angle) {
  //   pivotMotor.setPosition(angle);
  // }

  public void setArmPosition(TargetAction position) {
    this.goalPosition = clampPosition(position.getCoralArmAngle());
  }

  private Angle clampPosition(Angle pos) {
    if (ElevatorSubsystem.getInstance().getPosition() < ArmConstants.MIN_HP_ELEVATOR_HEIGHT
        && pos.in(Degrees) > TargetAction.HP.getCoralArmAngle().in(Degrees)) {

      System.out.println("DESIRED ANGLE PAST HP");
      return TargetAction.HP.getCoralArmAngle();
    }
    if (robotState.getHasCoral()) {
      if (pos.in(Degrees) < ArmConstants.MIN_CORAL_ANGLE.in(Degrees)) {
        System.out.println("DESIRED ANGLE BEYOND MIN LIMIT");
        return ArmConstants.MIN_CORAL_ANGLE;
      } else if (pos.in(Degrees) > ArmConstants.MAX_CORAL_ANGLE.in(Degrees)) {
        System.out.println("DESIRED ANGLE BEYOND MAX LIMIT");
        return ArmConstants.MAX_CORAL_ANGLE;
      }
    }

    return pos;
  }

  public Angle getArmDesiredPosition() {
    return goalPosition;
  }

  public Angle getArmAngle() {
    return pivotMotor.getPosition().getValue();
  }

  public boolean isAtDesiredPosition() {
    return isAtDesiredPosition(ArmConstants.DEG_TOL);
  }

  public boolean isAtDesiredPosition(double tol) {
    return isAtPosition(tol, goalPosition);
  }

  public boolean isAtPosition(double tol, Angle goal) {
    return pivotMotor.getPosition().getValueAsDouble() < goal.in(Degrees) + tol
        && pivotMotor.getPosition().getValueAsDouble() > goal.in(Degrees) - tol;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm Angle", pivotMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Arm Goal Angle", goalPosition.in(Degrees));
    // setArmAngle(goalPosition);
  }
}
