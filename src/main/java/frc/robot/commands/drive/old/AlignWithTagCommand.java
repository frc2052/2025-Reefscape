// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drive.old;

// import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.team2052.lib.planners.AutoAlignPlanner;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.RobotState;
// import frc.robot.commands.drive.DefaultDriveCommand;
// import frc.robot.commands.drive.SnapToLocationAngleCommand;
// import frc.robot.commands.drive.SnapToLocationAngleCommand.SnapLocation;
// import frc.robot.subsystems.vision.VisionSubsystem;
// import java.util.Optional;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import org.littletonrobotics.junction.Logger;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class AlignWithTagCommand extends DefaultDriveCommand {
//   private final VisionSubsystem vision = VisionSubsystem.getInstance();
//   private final RobotState robotState = RobotState.getInstance();
//   private PhotonTrackedTarget target;
//   private Translation2d targetTranslation;
//   private AlignLocation scoringLocation;
//   private AutoAlignPlanner planner;
//   private Timer targetTimer = new Timer();

//   private SnapLocation location;

//   private SwerveRequest.ApplyFieldSpeeds drive =
//       new SwerveRequest.ApplyFieldSpeeds()
//           .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

//   public AlignWithTagCommand(
//       AlignLocation scoringLocation,
//       DoubleSupplier xSupplier,
//       DoubleSupplier ySupplier,
//       DoubleSupplier rotationSupplier,
//       BooleanSupplier
//           fieldCentric) { // add enum supplier for scoring position, left middle or right
//     super(xSupplier, ySupplier, rotationSupplier, fieldCentric);

//     this.scoringLocation = scoringLocation;
//     planner = new AutoAlignPlanner();
//   }

//   @Override
//   public SwerveRequest getSwerveRequest() {
//     if (targetTranslation != null) {
//       // System.out.println(
//       //     "Camera Transform Rotation"
//       //         + target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
//       // return super.getSwerveRequest();
//       return drive.withSpeeds(
//           planner.calculate(
//               targetTranslation,
//               robotState
//                   .getFieldToRobot()
//                   .getTranslation()
//                   .plus(scoringLocation.goalTranslationFromTarget),
//
// Rotation2d.fromRadians(SnapToLocationAngleCommand.getLocationAngleRadians(location)),
//               robotState.getFieldToRobot()));
//     } else {
//       return super.getSwerveRequest();
//     }
//   }

//   @Override
//   public void execute() {
//     Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
//     if (tar.isPresent() && tar.get().getBestTarget().getBestCameraToTarget() != null) {
//       targetTimer.restart();
//       target = tar.get().getBestTarget();
//       location = getDirection();
//       targetTranslation =
//           robotState
//               .getFieldToRobot()
//               .getTranslation()
//               .plus(target.getBestCameraToTarget().getTranslation().toTranslation2d());

//     } else if (targetTimer.hasElapsed(0.2)) {
//       System.out.println("no target");
//       target = null;
//     }

//     Logger.recordOutput(
//         "REEF TAG TARGET",
//         new Pose2d(
//             targetTranslation,
//
// Rotation2d.fromRadians(SnapToLocationAngleCommand.getLocationAngleRadians(location))));
//     super.execute();
//   }

//   @Override
//   public boolean isFinished() {
//     if (planner.getAutoAlignComplete()) {
//       target = null;
//       location = null;
//       targetTranslation = null;
//       return true;
//     }
//     return false;
//   }

//   public SnapLocation getDirection() {
//     if (target == null) {
//       return location;
//     }
//     switch (target.fiducialId) {
//       case 18:
//         location = SnapLocation.ReefAB;
//         break;
//       case 17:
//         location = SnapLocation.ReefCD;
//         break;
//       case 22:
//         location = SnapLocation.ReefEF;
//         break;
//       case 21:
//         location = SnapLocation.ReefGH;
//         break;
//       case 20:
//         location = SnapLocation.ReefIJ;
//         break;
//       case 19:
//         location = SnapLocation.ReefKL;
//         break;
//       case 10:
//         location = SnapLocation.ReefGH;
//         break;
//       case 11:
//         location = SnapLocation.ReefIJ;
//         break;
//       case 6:
//         location = SnapLocation.ReefKL;
//         break;
//       case 7:
//         location = SnapLocation.ReefAB;
//         break;
//       case 8:
//         location = SnapLocation.ReefCD;
//         break;
//       case 9:
//         location = SnapLocation.ReefEF;
//         break;
//       default:
//         location = SnapLocation.FORWARD;
//     }

//     return location;
//   }

//   public enum AlignLocation {
//     LEFT(new Translation2d(0.2, 0.5)),
//     MIDDLE(new Translation2d(0.2, 0.1)),
//     RIGHT(new Translation2d(0.2, -0.5));

//     private Translation2d goalTranslationFromTarget;

//     private AlignLocation(Translation2d gt) {
//       this.goalTranslationFromTarget = gt;
//     }

//     public Translation2d getTranslation2d() {
//       return goalTranslationFromTarget;
//     }
//   }
// }
