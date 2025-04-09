// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team2052.lib.helpers.MathHelpers;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private static TalonFX frontMotor;
    private static TalonFX backMotor;

    private ControlState controlState;

    private boolean homing;
    private boolean shouldHome = true;
    private final DelayedBoolean homingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);

    private double goalPositionRotations;

    private static ElevatorSubsystem INSTANCE;

    public static ElevatorSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ElevatorSubsystem();
        }
        return INSTANCE;
    }

    private ElevatorSubsystem() {
        goalPositionRotations = TargetAction.HM.getElevatorPositionRotations();

        frontMotor = new TalonFX(Ports.ELEVATOR_FRONT_ID, "Krawlivore");
        backMotor = new TalonFX(Ports.ELEVATOR_BACK_ID, "Krawlivore");

        backMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
        frontMotor.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);

        frontMotor.clearStickyFault_SupplyCurrLimit();

        backMotor.setControl(new Follower(frontMotor.getDeviceID(), true));
    }

    public double clamp(double pos) {
        double armDeg = ArmPivotSubsystem.getInstance().getArmAngle().in(Degrees);
        if (armDeg > SuperstructureConstants.RIGHT_LIMIT - 2
                && armDeg < SuperstructureConstants.LEFT_LIMIT + 2) { // potentially danger zone
            if (getPosition() < SuperstructureConstants.MIN_SAFE_ROTATION) {
                pos = SuperstructureConstants.MIN_SAFE_ROTATION;
            }
        }

        return pos;
    }

    public void setPositionMotionMagic(TargetAction elevatorAction) {
        setPositionMotionMagic(elevatorAction.getElevatorPositionRotations());
    }

    public void setPositionMotionMagic(double elevatorPositionRotations) {
        controlState = ControlState.MOTION_MAGIC;
        if (goalPositionRotations == elevatorPositionRotations && atPosition()) {
            return;
        }

        if (elevatorPositionRotations != TargetAction.HM.getElevatorPositionRotations()) {
            shouldHome = true;
        }

        goalPositionRotations = elevatorPositionRotations;

        frontMotor.setControl(new MotionMagicExpoTorqueCurrentFOC(elevatorPositionRotations));
    }

    public void setOpenLoop(double speed) {
        controlState = ControlState.OPEN_LOOP;
        frontMotor.setControl(new DutyCycleOut(speed));
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
        return Math.abs(goalPositionRotations - getPosition()) <= ElevatorConstants.TICKS_DEADZONE;
    }

    public boolean atPosition(TargetAction position) {
        return Math.abs(position.getElevatorPositionRotations() - getPosition()) <= ElevatorConstants.TICKS_DEADZONE;
    }

    public boolean atPosition(double tol, TargetAction position) {
        return Math.abs(position.getElevatorPositionRotations() - getPosition()) <= tol;
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

    public boolean shouldHome() {
        return shouldHome;
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean atHomingLocation() {
        return getPosition() < TargetAction.HM.getElevatorPositionRotations()
                || MathHelpers.epsilonEquals(getPosition(), TargetAction.HM.getElevatorPositionRotations(), 0.05);
    }

    public void setNeutralMode(NeutralModeValue mode) {
        frontMotor.setNeutralMode(mode);
        backMotor.setNeutralMode(mode);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Elevator/Position", getPosition());
        Logger.recordOutput("Elevator/Goal Position", goalPositionRotations);

        if (DriverStation.isDisabled()) {
            goalPositionRotations = getPosition();
        }

        Logger.recordOutput("Elevator Homing", homing);
        if (homing) {
            setOpenLoop(ElevatorConstants.HOMING_SPEED);
            if (homingDelay.update(
                    Timer.getFPGATimestamp(),
                    MathHelpers.epsilonEquals(frontMotor.getVelocity().getValueAsDouble(), 0.0, 0.25))) {
                zeroEncoder();
                System.out.println("Elevator Homed");
                SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.STOW);
                homing = false;
                homingDelay.update(Timer.getFPGATimestamp(), false);
            }
        }
    }

    public static enum ControlState {
        OPEN_LOOP,
        MOTION_MAGIC
    }
}
