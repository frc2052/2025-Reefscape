// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.common.AutoFactory;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        if (isReal()) {
            // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        }

        Logger.start(); // Start logging

        Pose2d loadPose = FieldConstants.blueLeftBranches.get(0);
        if (loadPose != null) {
            System.out.println("Loaded Field Constants");
        }
    }

    @Override
    public void robotPeriodic() {
        RobotState.getInstance().output();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    public void forceRecompile() {
        AutoFactory.getInstance().recompile();
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.precompileAuto();
        m_robotContainer.precompileElevatorNudge();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.precompileElevatorNudge();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
