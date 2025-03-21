package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.io.Ports;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX motor;

    private static ClimberSubsystem INSTANCE;

    public static ClimberSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ClimberSubsystem();
        }
        return INSTANCE;
    }

    private ClimberSubsystem() {
        motor = new TalonFX(Ports.CLIMBER_ID, "Krawlivore");

        motor.getConfigurator().apply(ClimberConstants.MOTOR_CONFIG);
    }

    public void moveUp(boolean fineControl) {
        if (fineControl) {
            motor.set(ClimberConstants.fineSpeed);
        } else {
            motor.set(ClimberConstants.baseSpeed);
        }
    }

    public void moveDown(boolean fineControl) {
        if (fineControl) {
            motor.set(-ClimberConstants.fineSpeed);
        } else {
            motor.set(-ClimberConstants.baseSpeed);
        }
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getSpeed() {
        return motor.get();
    }

    public void setNeutralMode(NeutralModeValue mode) {
        motor.setNeutralMode(mode);
    }
}
