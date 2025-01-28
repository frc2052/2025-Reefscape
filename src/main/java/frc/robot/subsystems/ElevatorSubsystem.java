// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.Ports;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private static TalonFX frontMotor;
  private static TalonFX backMotor;

  private ControlState controlState;

  private boolean homing;
  private boolean shouldHome = false;
  private final DelayedBoolean homingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);

  private double goalPositionTicks;

  private static ElevatorSubsystem INSTANCE;

  public static ElevatorSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ElevatorSubsystem();
    }
    return INSTANCE;
  }

  private ElevatorSubsystem() {
    goalPositionTicks = ElevatorPosition.HOME.getPositionTicks();

    frontMotor = new TalonFX(Ports.ELEVATOR_FRONT_ID, "Krawlivore");
    backMotor = new TalonFX(Ports.ELEVATOR_BACK_ID, "Krawlivore");

    backMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
    frontMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);

    frontMotor.clearStickyFault_SupplyCurrLimit();

    backMotor.setControl(new Follower(frontMotor.getDeviceID(), true));
  }

  public void setPositionMotionMagic(ElevatorPosition elevatorPosition) {
    setPositionMotionMagic(elevatorPosition.getPositionTicks());
  }

  public void setPositionMotionMagic(double elevatorPositionTicks) {
    controlState = ControlState.MOTION_MAGIC;
    if (goalPositionTicks == elevatorPositionTicks && atPosition()) {
      return;
    }

    if (elevatorPositionTicks != ElevatorPosition.HOME.getPositionTicks()) {
      shouldHome = true;
    }

    goalPositionTicks = elevatorPositionTicks;

    frontMotor.setControl(new MotionMagicTorqueCurrentFOC(elevatorPositionTicks));
  }

  public void setOpenLoop(double speed) {
    controlState = ControlState.OPEN_LOOP;
    frontMotor.set(speed);
  }

  public Command manualUp() {
    return Commands.runOnce(() -> setOpenLoop(ElevatorConstants.MANUAL_MOTOR_SPEED), this);
  }

  public Command manualDown() {
    return Commands.runOnce(() -> setOpenLoop(-ElevatorConstants.MANUAL_MOTOR_SPEED), this);
  }

  public Command homeElevator() {
    return new InstantCommand(() -> setWantHome(true));
  }

  public Command stopElevator() {
    return new InstantCommand(() -> frontMotor.set(0.0));
  }

  public double getPosition() {
    return frontMotor.getPosition().getValueAsDouble();
  }

  public boolean atPosition() {
    return Math.abs(goalPositionTicks - frontMotor.getPosition().getValueAsDouble())
        <= ElevatorConstants.TICKS_DEADZONE;
  }

  public boolean atPosition(ElevatorPosition position) {
    return Math.abs(position.positionTicks - frontMotor.getPosition().getValueAsDouble())
        <= ElevatorConstants.TICKS_DEADZONE;
  }

  public void zeroEncoder() {
    frontMotor.getConfigurator().setPosition(0);
  }

  public void setWantHome(boolean home) {
    homing = home;
    // once homing is started, no longer needs to home
    if (homing) {
      shouldHome = false;
    }
  }

  public boolean atHomingLocation() {
    return getPosition() < ElevatorPosition.HOME.getPositionTicks()
        || MathHelpers.epsilonEquals(getPosition(), ElevatorPosition.HOME.getPositionTicks(), 0.05);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator Position", getPosition());
    Logger.recordOutput("Elevator Goal Position", goalPositionTicks);
    Logger.recordOutput("Elevator At Goal Position", atPosition());

    // if being used in open loop (usually manual mode), disable the height limit
    // if (controlState == ControlState.OPEN_LOOP) {
    //   frontMotor
    //       .getConfigurator()
    //       .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false));
    // } else {
    //   frontMotor
    //       .getConfigurator()
    //       .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true));
    // }

    // if we still intend to go to the home position, currently at alleged home, and should re-home,
    // then re-home
    if (MathHelpers.epsilonEquals(goalPositionTicks, ElevatorPosition.HOME.getPositionTicks(), .02)
        && atHomingLocation()
        && shouldHome) {
      setWantHome(true);
    } else if (controlState != ControlState.OPEN_LOOP) {
      setWantHome(false);
    }

    if (homing) {
      setOpenLoop(ElevatorConstants.HOMING_SPEED);
      if (homingDelay.update(
          Timer.getFPGATimestamp(),
          MathHelpers.epsilonEquals(frontMotor.getVelocity().getValueAsDouble(), 0.0, 0.01))) {
        zeroEncoder();
        setPositionMotionMagic(ElevatorPosition.HOME);
        homing = false;
        homingDelay.update(Timer.getFPGATimestamp(), false);
      }
    }
  }

  public static enum ElevatorPosition {
    HOME(2),
    HANDOFF(7),
    L1(10),
    L2(20),
    L3(37.5),
    L4(55),
    LOWER_ALGAE(25),
    UPPER_ALGAE(27),
    TRAVEL(5);

    private final double positionTicks;

    private ElevatorPosition(double positionTicks) {
      this.positionTicks = positionTicks;
    }

    public double getPositionTicks() {
      return positionTicks;
    }
  }

  public static enum ControlState {
    OPEN_LOOP,
    MOTION_MAGIC
  }
}
