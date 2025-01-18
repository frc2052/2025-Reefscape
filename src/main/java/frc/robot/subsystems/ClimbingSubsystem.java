package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; 
import frc.robot.Constants;



public class ClimbingSubsystem extends SubsystemBase {
    private final TalonFX climbingMotor;

    public ClimbingSubsystem() {
        climbingMotor = new TalonFX(Constants.MotorID.CLIMBER_MOTOR_ID);
    }

    //Up position is the upward moves the claw into a position for using the magnet on the cage
    public void moveUpPositionSpeed(int climbingSpeed) {
        climbingMotor.set(climbingSpeed);
    }
    //Down position is the default position and the postion needed to be moved in order to climb 
    public void moveDownPositionSpeed(int climbingSpeed) {
        climbingMotor.set(-climbingSpeed);
    }


}
