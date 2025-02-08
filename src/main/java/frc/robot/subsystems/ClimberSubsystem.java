package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private static ClimberSubsystem INSTANCE;

    public static ClimberSubsystem getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new ClimberSubsystem();
          }
          return INSTANCE;
    } 

    public ClimberSubsystem() {
        leadMotor = new TalonFX(ClimberConstants.Motors.LEAD_MOTOR_ID);
        followerMotor = new TalonFX(ClimberConstants.Motors.FOLLOW_MOTOR_ID);

        leadMotor.getConfigurator().apply(ClimberConstants.Configs.LEAD_MOTOR);

        followerMotor.setControl(new Follower(leadMotor.getDeviceID(), ClimberConstants.Configs.FOLLOW_MOTOR_OPPOSING_DIRECTION));
    }
    
    public void moveUp(boolean fineControl) {
        if (fineControl) {
            leadMotor.set(ClimberConstants.fineSpeed);
        } else {
            leadMotor.set(ClimberConstants.baseSpeed);
        }
    }

    public void moveDown(boolean fineControl) {
        if (fineControl) {
            leadMotor.set(-ClimberConstants.fineSpeed);
        } else {
            leadMotor.set(-ClimberConstants.baseSpeed);
        }
    }

    public void stopMotors() {
        leadMotor.stopMotor();
    }

    public double getSpeed() {
        return leadMotor.get();
    }
}