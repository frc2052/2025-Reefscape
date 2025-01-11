// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.io.Dashboard;

public class AlignWithTagCommand extends DefaultDriveCommand {
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final PIDController yController;
  private boolean lockedOnTag = false;
  public AlignWithTagCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) { // add enum supplier for scoring position, left middle or right
    super(xSupplier, ySupplier, rotationSupplier, () -> false);
    yController = new PIDController(1.75, 0, 0);
    yController.setTolerance(0.5);
  }

  @Override
  protected double getY() {
    if(!lockedOnTag) {
      return super.getY();
    } else {
      //put stuff here
      return 0;
    }
  }

  @Override
  protected double getRotation() {
    if(!lockedOnTag) {
      return super.getRotation();
    } else {
      //put stuff here for facing side of reef that tag is on
      return 0;
    }
  }

  @Override
  protected boolean getFieldCentric() {
    if(!lockedOnTag) {
      return super.getFieldCentric();
    } else {
      return false;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
