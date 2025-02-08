// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotState;
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
    goalPosition = ArmPosition.HANDOFF.getAngle();

    pivotMotor = new TalonFX(Ports.ARM_TALONFX_ID);

    pivotMotor.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);
  }

  private void setPivotAngle(Angle angle) {
    pivotMotor.setControl(new PositionTorqueCurrentFOC(angle).withSlot(0));
  }

  public void setArmPosition(ArmPosition position) {
    this.goalPosition = clampPosition(position.getAngle());
    setPivotAngle(goalPosition);
  }

  private Angle clampPosition(Angle pos) {
    if (ElevatorSubsystem.getInstance().getPosition() < ArmConstants.MIN_HP_ELEVATOR_HEIGHT
        && pos.in(Degrees) > ArmPosition.HANDOFF.getAngle().in(Degrees)) {
      System.out.println("DESIRED ANGLE PAST HP");
      return ArmPosition.HANDOFF.getAngle();
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
  }

  public enum ArmPosition { // TODO: set angles for each position
    HANDOFF(Degrees.of(300)),
    TRAVEL(Degrees.of(180)),
    L1(Degrees.of(110)),
    MID_LEVEL(Degrees.of(55)), // for both L2 and L3
    L4(Degrees.of(75)),
    UPPER_ALGAE_DESCORE(Degrees.of(90)),
    LOWER_ALGAE_DESCORE(Degrees.of(90));

    private final Angle angle;

    ArmPosition(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }
}
