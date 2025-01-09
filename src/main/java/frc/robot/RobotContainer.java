// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.AutoFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.drive.AdvantageScopeSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.util.Telemetry;
import frc.robot.util.io.Dashboard;

public class RobotContainer {
  private final ControlBoard controlBoard = ControlBoard.getInstance();
  public final Dashboard dashboard = Dashboard.getInstance();

  public final RobotState robotState = RobotState.getInstance();
  public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
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
    return new PathPlannerAuto("Example Auto");
    // return autoFactory.getCompiledAuto();
  }
}