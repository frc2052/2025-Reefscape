// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.common.AutoFactory;
import frc.robot.commands.algae.AlgaeCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.DriveWhilePointingAtReefCommand;
import frc.robot.commands.drive.SnapToLocationAngleCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.drive.auto.AutoSnapToLocationAngleCommand;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.AlgaePivotSubsystem;
import frc.robot.subsystems.AlgaeShooterSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.TargetFieldLocation;
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
  public final AlgaeShooterSubsystem algaeShooter = AlgaeShooterSubsystem.getInstance();
  public final AlgaePivotSubsystem algaePivot = AlgaePivotSubsystem.getInstance();
  public final LedSubsystem leds = LedSubsystem.getInstance();

  public static boolean deadReckoning = false;

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
    /* Primary Driver */
    configurePOVBindings();

    drivetrain.registerTelemetry(logger::telemeterize);

    controlBoard.resetGyro().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

    controlBoard.intake().whileTrue(HandCommandFactory.motorIn());
    controlBoard
        .outtake()
        .whileTrue(HandCommandFactory.motorOut())
        .onFalse(
            new InstantCommand(
                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HP)));

    controlBoard.intakeAlgae().whileTrue(AlgaeCommandFactory.intake());
    controlBoard.outtakeAlgae().whileTrue(AlgaeCommandFactory.outtake());

    controlBoard
        .alignWithElement()
        .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(robotState::getAlignOffset));
    controlBoard
        .pointToReef()
        .whileTrue(
            new DriveWhilePointingAtReefCommand(
                controlBoard::getThrottle, controlBoard::getStrafe, dashboard::isFieldCentric));

    controlBoard
        .povRotLeft()
        .onTrue(
            new InstantCommand(() -> robotState.setAlignOffset(AlignOffset.LEFT_CORAL_STATION_LOC)))
        .whileTrue(
            new SnapToLocationAngleCommand(
                TargetFieldLocation.LCS,
                controlBoard::getThrottle,
                // Sideways velocity supplier.
                controlBoard::getStrafe,
                // Rotation velocity supplier.
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    controlBoard
        .povRotRight()
        .onTrue(
            new InstantCommand(
                () -> robotState.setAlignOffset(AlignOffset.RIGHT_CORAL_STATION_LOC)))
        .whileTrue(
            new SnapToLocationAngleCommand(
                TargetFieldLocation.RCS,
                controlBoard::getThrottle,
                // Sideways velocity supplier.
                controlBoard::getStrafe,
                // Rotation velocity supplier.
                controlBoard::getRotation,
                dashboard::isFieldCentric));

    /* Secondary Driver */
    controlBoard
        .actTrigger()
        .onTrue(new InstantCommand(() -> superstructure.confirmSelectedAction()));

    controlBoard
        .setGoalCL()
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setSelectedTargetAction(TargetAction.CL, true);

                  robotState.setAlignOffset(AlignOffset.MIDDLE_REEF_LOC);
                }));
    controlBoard
        .setGoalL1H()
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setSelectedTargetAction(TargetAction.L1H, false);

                  robotState.setAlignOffset(AlignOffset.MIDDLE_REEF_LOC);
                }));
    controlBoard
        .setGoalL2()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setSelectedTargetAction(TargetAction.L2, false)));
    controlBoard
        .setGoalL3()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setSelectedTargetAction(TargetAction.L3, false)));
    controlBoard
        .setGoalL4()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setSelectedTargetAction(TargetAction.L4, false)));
    controlBoard
        .setGoalLowerAlgae()
        .onTrue(
            new InstantCommand(
                () -> AlgaePivotSubsystem.getInstance().setPivotAngle(Degrees.of(180))))
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setSelectedTargetAction(TargetAction.LA, false);

                  robotState.setAlignOffset(AlignOffset.MIDDLE_REEF_LOC);
                }));
    controlBoard
        .setGoalUpperAlgae()
        .onTrue(
            new InstantCommand(
                () -> AlgaePivotSubsystem.getInstance().setPivotAngle(Degrees.of(180))))
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setSelectedTargetAction(TargetAction.UA, false);

                  robotState.setAlignOffset(AlignOffset.MIDDLE_REEF_LOC);
                }));
    controlBoard
        .setGoalCoralStation()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setSelectedTargetAction(TargetAction.HP, false)));
    controlBoard
        .homeElevator()
        .onTrue(
            new InstantCommand(
                () -> superstructure.setSelectedTargetAction(TargetAction.HM, false)));

    controlBoard
        .setSubReefLeft()
        .onTrue(new InstantCommand(() -> robotState.setAlignOffset(AlignOffset.LEFT_REEF_LOC)));
    controlBoard
        .setSubReefRight()
        .onTrue(new InstantCommand(() -> robotState.setAlignOffset(AlignOffset.RIGHT_REEF_LOC)));

    controlBoard
        .algaeScoreAngle()
        .onTrue(
            new InstantCommand(
                () -> AlgaePivotSubsystem.getInstance().setPivotAngle(Degrees.of(250))));
    controlBoard
        .algaeLowAngle()
        .onTrue(
            new InstantCommand(
                () -> AlgaePivotSubsystem.getInstance().setPivotAngle(Degrees.of(135))));

    controlBoard
        .algaeManualUp()
        .onTrue(
            new InstantCommand(() -> AlgaePivotSubsystem.getInstance().setPivotSpeedManual(0.2)))
        .onFalse(
            new InstantCommand(() -> AlgaePivotSubsystem.getInstance().setPivotSpeedManual(0.0)));
    controlBoard
        .algaeManualDown()
        .onTrue(
            new InstantCommand(() -> AlgaePivotSubsystem.getInstance().setPivotSpeedManual(-0.2)))
        .onFalse(
            new InstantCommand(() -> AlgaePivotSubsystem.getInstance().setPivotSpeedManual(0.0)));

    controlBoard.climbUp().whileTrue(ClimberCommandFactory.climberUp());
    controlBoard.climbDown().whileTrue(ClimberCommandFactory.climberDown());

    // controlBoard.sysIDQuasiForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // controlBoard.sysIDQuasiReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // controlBoard.sysIDDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // controlBoard.sysIDDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // controlBoard
    //     .shoot()
    //     .onTrue(Commands.runOnce(SignalLogger::start))
    //     .onFalse(Commands.runOnce(SignalLogger::stop));

    // controlBoard
    //     .manualUp()
    //     .onTrue(ElevatorSubsystem.getInstance().manualUp())
    //     .onFalse(ElevatorSubsystem.getInstance().stopElevator());

    // controlBoard
    //     .manualDown()
    //     .onTrue(ElevatorSubsystem.getInstance().manualDown())
    //     .onFalse(ElevatorSubsystem.getInstance().stopElevator());
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
        .whileTrue(new DefaultDriveCommand(() -> 0.0, () -> -0.2, () -> 0.0, () -> false));

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
        .whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.2, () -> 0.0, () -> false));

    controlBoard
        .povUpLeft()
        .whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.2, () -> 0.0, () -> false));

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

  public static boolean getDeadReckoning() {
    return deadReckoning;
  }

  public static void setDeadReckoning(boolean isDeadReckoning) {
    deadReckoning = isDeadReckoning;
  }
}
