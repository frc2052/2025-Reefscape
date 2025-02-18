package frc.robot.commands.drive;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CircleAroundReefCommand extends Command {

  private DefaultDriveCommand defaultDriveCommand;
  private DrivetrainSubsystem drivetrain;
  private Timer timer;
  private boolean isMovingRight;
  private RobotState robotState = RobotState.getInstance();

  private double radius;
  private double Speed;

  public CircleAroundReefCommand(double radius,double Speed,boolean isMovingRight,DrivetrainSubsystem drivetrain) {
    this.radius = radius;
    this.Speed = Speed;
    this.drivetrain = drivetrain;


    timer.start();

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double xVelocity = radius * Math.cos(Speed * timer.getFPGATimestamp()); 
    double yVelocity = radius * Math.sin(Speed * timer.getFPGATimestamp());  
    double heading = Math.atan2(robotState.getFieldToRobot().getY() - signChanger(yVelocity), robotState.getFieldToRobot().getX() - xVelocity);
    DoubleSupplier xVelocitySupplier = () -> signChanger(xVelocity);
    DoubleSupplier yVelocitySupplier = () -> signChanger(yVelocity);
    DoubleSupplier rotationVelocitySupplier = () -> heading;

   new DefaultDriveCommand(xVelocitySupplier,yVelocitySupplier,rotationVelocitySupplier, ()-> false);
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
    timer.stop();
    defaultDriveCommand.end(interrupted);
  }   

  @Override
  public boolean isFinished() {
    return false;
  }
}
