// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmRollerConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class ArmRollerSubsystem extends SubsystemBase {
    private final TalonFX motor;
    // private final CANrange range;
    private static ArmRollerSubsystem INSTANCE;
    private boolean isIntaking = false;

    public static ArmRollerSubsystem getInstance() {
        if (INSTANCE == null) {
            return new ArmRollerSubsystem();
        }
        return INSTANCE;
    }

    public ArmRollerSubsystem() {
        motor = new TalonFX(Ports.HAND_TALONFX_ID);
        // range = new CANrange(Ports.HAND_CAN_RANGE);

        // range.getConfigurator().apply(HandConstants.CANRANGE_CONFIG);

        motor.getConfigurator().apply(ArmRollerConstants.MOTOR_CONFIG);
    }

    private void setMotor(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        isIntaking = false;
        motor.stopMotor();
    }

    public void coralOut() {
        setMotor(
                SuperstructureSubsystem.getInstance().getCurrentAction().equals(TargetAction.L1H)
                        ? Constants.ArmRollerConstants.CORAL_L1_OUT_SPEED
                        : Constants.ArmRollerConstants.CORAL_OUT_SPEED);
    }

    public void coralIn() {
        isIntaking = true;
        setMotor(-Constants.ArmRollerConstants.CORAL_IN_SPEED);
    }

    public void algaeOut() {
        setMotor(ArmRollerConstants.ALGAE_OUT_SPEED);
    }

    public void algaeIn() {
        setMotor(-Constants.ArmRollerConstants.ALGAE_IN_SPEED);
    }

    public double motorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public boolean getHasCoral() {
        // return range.getIsDetected().getValue();
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.recordOutput("Hand/Motor Velocity", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Hand/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        // Logger.recordOutput("Hand/Has Coral", getHasCoral());
        // Logger.recordOutput("Hand/ToF Distance", range.getDistance().getValueAsDouble());
        RobotState.getInstance().setHasCoral(getHasCoral());
        RobotState.getInstance().setIsIntaking(isIntaking);
    }
}
