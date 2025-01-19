// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final TalonSRX rollerMotor;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new TalonSRX(RollerConstants.ROLLER_MOTOR_ID);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse) {
    double forwardSpeed = forward * RollerConstants.ROLLER_IN_SPEED;
    double reverseSpeed = reverse * RollerConstants.ROLLER_OUT_SPEED;
    rollerMotor.set(TalonSRXControlMode.PercentOutput,forwardSpeed - reverseSpeed);
  }
}
