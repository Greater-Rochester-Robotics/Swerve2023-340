// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ADIS16470_IMU.IMUAxis;


public class DriveSetGyro extends InstantCommand {
  private double angle;
  IMUAxis axis;

  public DriveSetGyro(double angle) {
    this(angle,IMUAxis.kYaw);
  }

  /**
   * Set the gyro's current angle to the input 
   * param. CCW is positive.
   * 
   * @param angle an angle in DEGREES!!!
   */
  public DriveSetGyro(double angle, IMUAxis axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    this.angle = angle;
    this.axis = axis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (axis) {
      case kYaw: case kZ:
        RobotContainer.swerveDrive.setGyroYawAngle(angle);
        break;
      case kPitch: case kY:
        RobotContainer.swerveDrive.setGyroPitchAngle(angle);
        break;
      case kRoll: case kX:
        RobotContainer.swerveDrive.setGyroRollAngle(angle);
        break;
    }
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
