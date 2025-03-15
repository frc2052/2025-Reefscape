// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.io.Ports;

public class IntakePivotSubsystem extends SubsystemBase {
    private TalonFX pivotMotor;
    private CANcoder encoder;
    /** Public method to provide access to the instance. */
    private static IntakePivotSubsystem INSTANCE;

    public static IntakePivotSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakePivotSubsystem();
        }
        return INSTANCE;
    }

    private IntakePivotSubsystem() {
        pivotMotor = new TalonFX(Ports.INTAKE_PIVOT_ID);
        encoder = new CANcoder(Ports.INTAKE_ENCODER_ID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
