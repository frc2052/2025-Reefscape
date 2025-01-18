package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;



public class ClimbingSubsystem extends SubsystemBase {
    private final TalonFX climbingMotor;
    private boolean restingStatus = true;
    private static ClimbingSubsystem INSTANCE;
    

    public ClimbingSubsystem() {
        climbingMotor = new TalonFX(Constants.MotorID.CLIMBER_MOTOR_ID);
        InvertedValue climbingMotorInverted =
        Constants.ClimberConstants.climberMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    TalonFXConfiguration climbingMotorConfig = new TalonFXConfiguration();
    climbingMotorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(climbingMotorInverted).withNeutralMode(NeutralModeValue.Brake));

    }
    public static ClimbingSubsystem getInstance(){
        if (INSTANCE == null) {
            INSTANCE = new ClimbingSubsystem();
          }
          return INSTANCE;
    } 
    
    //Up position is the upward moves the claw into a position for using the magnet on the cage
    public void moveUpPositionSpeed(int climbingSpeed) {
        climbingMotor.set(climbingSpeed);
        restingStatus = false;

    }
    //Down position is the default position and the postion needed to be moved in order to climb 
    public void moveDownPositionSpeed(int climbingSpeed) {
        climbingMotor.set(-climbingSpeed);
        restingStatus = true;
    }

    public void stopMotors() {
        climbingMotor.set(0);
    }

    public boolean getRestingStatus() {
        return restingStatus;
    }


}
