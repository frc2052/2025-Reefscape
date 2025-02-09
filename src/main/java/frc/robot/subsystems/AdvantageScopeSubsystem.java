// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DrivetrainSubsystem;

public class AdvantageScopeSubsystem extends SubsystemBase {
  static DrivetrainSubsystem drivetrainSubsystem;

  static String folder = "Data_";

  private static AdvantageScopeSubsystem INSTANCE;

  public static AdvantageScopeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AdvantageScopeSubsystem(drivetrainSubsystem);
    }
    return INSTANCE;
  }

  public AdvantageScopeSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
    AdvantageScopeSubsystem.drivetrainSubsystem = drivetrainSubsystem;
  }

  @Override
  public void periodic() {
    recordDrivetrainData();
  }

  public static void recordDrivetrainData() {
  }
}
