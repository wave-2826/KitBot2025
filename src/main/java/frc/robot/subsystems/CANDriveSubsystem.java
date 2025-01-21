// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANDriveSubsystem extends SubsystemBase {
  private final TalonSRX leftLeader;
  private final TalonSRX leftFollower;
  private final TalonSRX rightLeader;
  private final TalonSRX rightFollower;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new TalonSRX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new TalonSRX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new TalonSRX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new TalonSRX(DriveConstants.RIGHT_FOLLOWER_ID);
  }

  public void tankDrive(double leftMotors, double rightMotors) {
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftMotors * DriveConstants.MOTOR_SPEED);
    leftFollower.set(TalonSRXControlMode.PercentOutput, leftMotors * DriveConstants.MOTOR_SPEED);
    
    rightLeader.set(TalonSRXControlMode.PercentOutput, -rightMotors * DriveConstants.MOTOR_SPEED);
    rightFollower.set(TalonSRXControlMode.PercentOutput, -rightMotors * DriveConstants.MOTOR_SPEED);
  }

  public void DiffDrive(double stickY, double stickX) {
    // Apply deadband
    if (Math.abs(stickY) < 0.1) {
        stickY = 0;
    }
    if (Math.abs(stickX) < 0.1) {
        stickX = 0;
    }

    // Calculate left and right motor outputs
    double leftOutput = stickX + stickY;
    double rightOutput = stickX - stickY;

    // Scale outputs to be within the range of -1 to 1
    leftOutput = Math.max(-1, Math.min(1, leftOutput * DriveConstants.MOTOR_SPEED));
    rightOutput = Math.max(-1, Math.min(1, rightOutput * DriveConstants.MOTOR_SPEED));

    // Set motor outputs
    leftLeader.set(TalonSRXControlMode.PercentOutput, leftOutput);
    leftFollower.set(TalonSRXControlMode.PercentOutput, leftOutput);
    rightLeader.set(TalonSRXControlMode.PercentOutput, rightOutput);
    rightFollower.set(TalonSRXControlMode.PercentOutput, rightOutput);
}


  @Override
  public void periodic() {

  }
}
