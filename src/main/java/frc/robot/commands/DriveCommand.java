// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.DriveConstants;


// Command to drive the robot with joystick inputs
public class DriveCommand extends Command {
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightY;
  private final DoubleSupplier leftX;
  private final CANDriveSubsystem driveSubsystem;

  // Constructor. Runs only once when the command is first created.
  public DriveCommand(DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier leftX, CANDriveSubsystem driveSubsystem) {
    // Save parameters to local variables for use later
    this.leftY = leftY;
    this.rightY = rightY;
    this.leftX = leftX;
    this.driveSubsystem = driveSubsystem;

    // Declare subsystems required by this command. This should include any
    // subsystem this command sets and output of
    addRequirements(this.driveSubsystem);
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    if (DriveConstants.DRIVE_TYPE) {
      driveSubsystem.DiffDrive(leftY.getAsDouble(), leftX.getAsDouble());
    }
    else {
      driveSubsystem.tankDrive(leftY.getAsDouble(), rightY.getAsDouble());
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
