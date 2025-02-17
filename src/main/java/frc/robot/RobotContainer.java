// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.common.AutoFactory;
import frc.robot.commands.algae.AlgaeCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.auto.AutoSnapToLocationAngleCommand;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.PositionSuperstructure.TargetFieldLocation;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Telemetry;
import frc.robot.util.io.Dashboard;

public class RobotContainer {
  private final ControlBoard controlBoard = ControlBoard.getInstance();
  public final Dashboard dashboard = Dashboard.getInstance();

  public final RobotState robotState = RobotState.getInstance();
  public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
  public final VisionSubsystem vision = VisionSubsystem.getInstance();
  public final AdvantageScopeSubsystem advantageScope = AdvantageScopeSubsystem.getInstance();
  public final AutoFactory autoFactory = AutoFactory.getInstance();
  public final SuperstructureSubsystem superstructure = SuperstructureSubsystem.getInstance();

  private final Telemetry logger =
      new Telemetry(DrivetrainConstants.DRIVE_MAX_SPEED.in(MetersPerSecond));

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        new DefaultDriveCommand(
            controlBoard::getThrottle,
            // Sideways velocity supplier.
            controlBoard::getStrafe,
            // Rotation velocity supplier.
            controlBoard::getRotation,
            dashboard::isFieldCentric));

    configureNamedCommands();

    configureBindings();
  }

  private void configureNamedCommands() {
    // snap to angle in gui
    NamedCommands.registerCommand(
        "Snap Left Coral Station", new AutoSnapToLocationAngleCommand(TargetFieldLocation.LCS));
    NamedCommands.registerCommand(
        "Snap Right Coral Station", new AutoSnapToLocationAngleCommand(TargetFieldLocation.RCS));

    NamedCommands.registerCommand(
        "Snap to AB", new AutoSnapToLocationAngleCommand(TargetFieldLocation.AB));
    NamedCommands.registerCommand(
        "Snap to CD", new AutoSnapToLocationAngleCommand(TargetFieldLocation.CD));
    NamedCommands.registerCommand(
        "Snap to EF", new AutoSnapToLocationAngleCommand(TargetFieldLocation.EF));
    NamedCommands.registerCommand(
        "Snap to GH", new AutoSnapToLocationAngleCommand(TargetFieldLocation.GH));
    NamedCommands.registerCommand(
        "Snap to IJ", new AutoSnapToLocationAngleCommand(TargetFieldLocation.IJ));
    NamedCommands.registerCommand(
        "Snap to KL", new AutoSnapToLocationAngleCommand(TargetFieldLocation.KL));

    // // score in gui
    // NamedCommands.registerCommand("Score L1", ToLevel.L1.getCommand());
    // NamedCommands.registerCommand("Score L2", ToLevel.L2.getCommand());
    // NamedCommands.registerCommand("Score L3", ToLevel.L3.getCommand());
    // NamedCommands.registerCommand("Score L4", ToLevel.L4.getCommand());
    // NamedCommands.registerCommand("Score Processor", null); // TODO: auto processor score
    // NamedCommands.registerCommand("DeScore Algae", null); // TODO: auto descore algae
    // NamedCommands.registerCommand("Coral Station Intake", null); // TODO: auto coral station
    // intake
  }

  private void configureBindings() {
    configurePOVBindings();

    drivetrain.registerTelemetry(logger::telemeterize);

    controlBoard
        .resetGyro()
        .onTrue(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d())));

    // controlBoard
    //     .reefAlignment() // 3
    //     .whileTrue(
    //         new AlignWithReefCommand(
    //             () -> ReefSubSide.CENTER,
    //             controlBoard::getThrottle,
    //             // Sideways velocity supplier.
    //             controlBoard::getStrafe,
    //             // Rotation velocity supplier.
    //             controlBoard::getRotation,
    //             dashboard::isFieldCentric));

    // controlBoard.sysIDQuasiForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // controlBoard.sysIDQuasiReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // controlBoard.sysIDDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // controlBoard.sysIDDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // controlBoard
    //     .shoot()
    //     .onTrue(Commands.runOnce(SignalLogger::start))
    //     .onFalse(Commands.runOnce(SignalLogger::stop));
    controlBoard.intakeCoral().whileTrue(HandCommandFactory.motorIn());
    controlBoard.outtakeCoral().whileTrue(HandCommandFactory.motorOut());

    controlBoard.intakeAlgae().whileTrue(AlgaeCommandFactory.intake());
    controlBoard.shootAlgae().whileTrue(AlgaeCommandFactory.score());

    controlBoard
        .manualUp()
        .onTrue(AlgaeSubsystem.getInstance().runPivotPct(0.40))
        .onFalse(AlgaeSubsystem.getInstance().runPivotPct(0.0));
    controlBoard
        .manualDown()
        .onTrue(AlgaeSubsystem.getInstance().runPivotPct(-0.40))
        .onFalse(AlgaeSubsystem.getInstance().runPivotPct(0.0));

    // controlBoard
    //     .manualUp()
    //     .onTrue(ElevatorSubsystem.getInstance().manualUp())
    //     .onFalse(ElevatorSubsystem.getInstance().stopElevator());

    // controlBoard
    //     .manualDown()
    //     .onTrue(ElevatorSubsystem.getInstance().manualDown())
    //     .onFalse(ElevatorSubsystem.getInstance().stopElevator());

    controlBoard.climbUp().whileTrue(ClimberCommandFactory.climberUp());

    controlBoard.climbDown().whileTrue(ClimberCommandFactory.climberDown());
  }

  private void configurePOVBindings() {
    ControlBoard controlBoard = ControlBoard.getInstance();

    controlBoard
        .povUp()
        .whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.0, () -> 0.0, () -> false));
    controlBoard
        .povUpRight()
        .whileTrue(new DefaultDriveCommand(() -> 0.2, () -> -0.2, () -> 0.0, () -> false));

    controlBoard
        .povRight()
        .whileTrue(new moveAroundReef(getdistance(fieldLocation.Reef.getPose()),1,true));

    controlBoard
        .povDownRight()
        .whileTrue(new DefaultDriveCommand(() -> -0.2, () -> -0.2, () -> 0.0, () -> false));

    controlBoard
        .povDown()
        .whileTrue(new DefaultDriveCommand(() -> -0.2, () -> 0.0, () -> 0.0, () -> false));

    controlBoard
        .povDownLeft()
        .whileTrue(new DefaultDriveCommand(() -> -0.2, () -> 0.2, () -> 0.0, () -> false));

    controlBoard
        .povLeft()
        .whileTrue(new moveAroundReef(getdistance(fieldLocation.Reef.getPose()),1,false));// adjust the following values

    controlBoard
        .povUpLeft()
        .whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.2, () -> 0.0, () -> false));

    controlBoard
        .povRotLeft()
        .whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.0, () -> 0.3, () -> false));

    controlBoard
        .povRotRight()
        .whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.0, () -> -0.3, () -> false));

    System.out.println("POV Bindings Configured");
  }

  public void forceRecompile() {
    autoFactory.recompile();
  }

  public void precompileAuto() {
    if (autoFactory.recompileNeeded()) {
      autoFactory.recompile();
    }
  }

  public Command getAutonomousCommand() {
    return autoFactory.getCompiledAuto();
  }

  private double getdistance(Transform2d reef, Pose2d robot){
    if (fieldLocation == fieldLocation.Reef){
        robot = robotState.getFieldToRobot();
        private Translation2d robotTranslation = robotPose.getTranslation();
        return robotTranslation.getDistance(reef);
    }else{
        return;
    }
  }
}
