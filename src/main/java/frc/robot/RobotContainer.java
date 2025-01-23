// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.common.AutoFactory;
import frc.robot.commands.drive.AlignWithReefCommand;
import frc.robot.commands.drive.AlignWithReefCommand.AlignLocation;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.SnapToLocationAngleCommand;
import frc.robot.commands.elevator.ElevatorCommandFactory;
import frc.robot.commands.superstructure.SuperstructureCommandFactory;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.drive.AdvantageScopeSubsystem;
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

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.registerTelemetry(logger::telemeterize);
    controlBoard.resetGyro().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));
    configurePOVBindings();
    controlBoard
        .reefAlignment()
        .whileTrue(
            new AlignWithReefCommand(
                () -> AlignLocation.MIDDLE,
                controlBoard::getThrottle,
                // Sideways velocity supplier.
                controlBoard::getStrafe,
                // Rotation velocity supplier.
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard.sysIDQuasiForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    controlBoard.sysIDQuasiReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    controlBoard.sysIDDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    controlBoard.sysIDDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    controlBoard
        .shoot()
        .onTrue(Commands.runOnce(SignalLogger::start))
        .onFalse(Commands.runOnce(SignalLogger::stop));
  }

  private void configurePOVBindings() {
    ControlBoard controlBoard = ControlBoard.getInstance();

    controlBoard
        .povUp()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.ReefGH,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povUpRight()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.ReefEF,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povRight()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.RightCoralStation,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povDownRight()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.ReefCD,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povDown()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.ReefAB,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povDownLeft()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.ReefKL,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povLeft()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.LeftCoralStation,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povUpLeft()
        .whileTrue(
            new SnapToLocationAngleCommand(
                SnapToLocationAngleCommand.SnapLocation.ReefIJ,
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    System.out.println("POV Bindings Configured");

    controlBoard.manualUp().whileTrue(ElevatorCommandFactory.manualUp());

    controlBoard.manualDown().whileTrue(ElevatorCommandFactory.manualDown());

    controlBoard
        .setElevatorPositionL1()
        .whileTrue(SuperstructureCommandFactory.ToLevel.L1.getCommand());
    controlBoard
        .setElevatorPositionL2()
        .whileTrue(SuperstructureCommandFactory.ToLevel.L2.getCommand());
    controlBoard
        .setElevatorPositionL3()
        .whileTrue(SuperstructureCommandFactory.ToLevel.L3.getCommand());
    controlBoard
        .setElevatorPositionL4()
        .whileTrue(SuperstructureCommandFactory.ToLevel.L4.getCommand());
    controlBoard
        .setElevatorPositionUpperAlgae()
        .whileTrue(SuperstructureCommandFactory.ToLevel.DESCORE_HIGH_ALGAE.getCommand());
    controlBoard
        .setElevatorPositionLowerAlgae()
        .whileTrue(SuperstructureCommandFactory.ToLevel.DESCORE_LOW_ALGAE.getCommand());
    controlBoard
        .setElevatorPositionHandoff()
        .whileTrue(SuperstructureCommandFactory.ToLevel.HANDOFF.getCommand());
    controlBoard
        .setElevatorPositionTravel()
        .whileTrue(SuperstructureCommandFactory.ToLevel.TRAVEL.getCommand());
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
    return new PathPlannerAuto("New Auto");
    // return autoFactory.getCompiledAuto();
  }
}
