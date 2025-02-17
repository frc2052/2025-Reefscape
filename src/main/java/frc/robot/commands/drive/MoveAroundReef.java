package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CircleAroundObjectCommand extends CommandBase {

  private DefaultDriveCommand defaultDriveCommand;
  private DrivetrainSubsystem drivetrain;
  private Timer timer;

  private double radius;
  private double angularSpeed;

  public CircleAroundObjectCommand extends Command(double radius,double Speed,boolean isMovingRight) {
    this.radius = radius;
    this.angularSpeed = angularSpeed;

    defaultDriveCommand = DefaultDriveCommand.getInstance();
    timer.start()

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double xVelocity = radius * Math.cos(Speed * timer.getFPGATimeStamp()); 
    double yVelocity = radius * Math.sin(Speed * timer.getFPGATimeStamp());  
    double heading = Math.atan2(robotState.getFieldToRobot().getY() - signChanger(yVelocity), robotState.getFieldToRobot().getX() - xVelocity);
    DoubleSupplier xVelocitySupplier = () -> signChanger(xVelocity);
    DoubleSupplier yVelocitySupplier = () -> signChanger(yVelocity);
    DoubleSupplier rotationVelocitySupplier = () -> heading;

   defaultDriveCommand(xVelocitySupplier,yVelocitySupplier,rotationVelocitySupplier);
  }

  private double signChanger (double x){
    if (isMovingRight){
      return -x;
    }else{
      return x;
    }
  } 
  
  @Override
  public void end(boolean interrupted) {
    timer.stop()
    defaultDriveCommand.end(interrupted);
  }   

  @Override
  public boolean isFinished() {
    return false;
  }
}
