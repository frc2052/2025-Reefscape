package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CircleAroundObjectCommand extends CommandBase {

  private final DefaultDriveCommand defaultDriveCommand;
  private final DrivetrainSubsystem drivetrain;
  private Timer timer;

  private final double radius;
  private final double angularSpeed;

  public CircleAroundObjectCommand extends Command(
      double radius,
      double angularSpeed) {
    this.radius = radius;
    this.angularSpeed = angularSpeed;

    defaultDriveCommand = DefaultDriveCommand.getInstance();
    timer.start()

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double xVelocity = radius * Math.cos(angularSpeed * time); 
    double yVelocity = radius * Math.sin(angularSpeed * time);  
    double heading = Math.atan2(robotState.getFieldToRobot().getY() - yVelocity, robotState.getFieldToRobot().getX() - xVelocity);
    DoubleSupplier xVelocitySupplier = () -> xVelocity;
    DoubleSupplier yVelocitySupplier = () -> yVelocity;
    DoubleSupplier rotationVelocitySupplier = () -> heading;

    defaultDriveCommand(xVelocitySupplier, yVelocitySupplier, rotationVelocitySupplier);
  }
  
  @Override
  public void end(boolean interrupted) {
    timer.stop()
    defaultDriveCommand.end(interrupted);
  }   

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
