// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.io.Ports;

public class IntakeRollerSubsystem extends SubsystemBase {
    private TalonFX intakeMotor;

    /** Public method to provide access to the instance. */
    private static IntakeRollerSubsystem INSTANCE;

    public static IntakeRollerSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeRollerSubsystem();
        }
        return INSTANCE;
    }

    private IntakeRollerSubsystem() {
        intakeMotor = new TalonFX(Ports.INTAKE_ROLLER_ID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
