package frc.robot.commands.drive.alignment;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import java.util.function.Supplier;

public class DriveToPosePLoop extends Command {
    private final double driveP = 0.77;
    private final double driveMaxSpeed = 1.0;
    private final double driveMaxAcceleration = 4.0;
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final Supplier<Pose2d> target;

    //     private final SwerveRequest.ApplyRobotSpeeds driveChassisSpeeds = new SwerveRequest.ApplyRobotSpeeds()
    //             .withDesaturateWheelSpeeds(true)
    //             .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentricFacingAngle driveChassisSpeeds = new SwerveRequest.RobotCentricFacingAngle()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withHeadingPID(3.5, 0, 0);
    private Supplier<Pose2d> robot = RobotState.getInstance()::getFieldToRobot;

    private Pose2d targetPose;

    public DriveToPosePLoop(Supplier<Pose2d> target) {
        this.target = target;

        addRequirements(drivetrain);
    }

    public DriveToPosePLoop(Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        this(target);
        this.robot = robot;
    }

    @Override
    public void initialize() {
        targetPose = null;
    }

    @Override
    public void execute() {
        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        if (target.get() != null) {
            targetPose = target.get();
        }

        if (targetPose == null) {
            System.out.println(this.getName() + " has no target pose");
            return;
        }

        double alignX = driveP * -(targetPose.getY() - currentPose.getY());
        double alignY = driveP * (targetPose.getX() - currentPose.getX());

        drivetrain.setControl(driveChassisSpeeds
                .withTargetDirection(targetPose.getRotation())
                .withVelocityX(alignX)
                .withVelocityY(alignY));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
