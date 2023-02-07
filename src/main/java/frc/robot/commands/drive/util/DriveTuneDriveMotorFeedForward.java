// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.SwerveModNum;

/**
 * A testing command meant to spin the robot at a specific 
 * duty cycle, allowing data from drive motors without need 
 * for a full field. The resulting speed can be used to 
 * compute the feedforward for the swerve Move motor
 * controllers. The robot should spin in place.
 * 
 */
public class DriveTuneDriveMotorFeedForward extends CommandBase {
  private double speed = .75;
  private double[] angle = new double[]{Math.toRadians(135), Math.toRadians(-135), Math.toRadians(-45), Math.toRadians(45)};
  /** Creates a new DriveTuneDriveMotorFeedForward. */
  public DriveTuneDriveMotorFeedForward(double speed) {
    this.speed = speed;
    // Use addRequirements() here to use subsystem.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //use driveOneModule for each motor, position setting m0 to 135, m1 to -135, m2 to -45, and m3 to 45, set speed to field speed
    for (int i=0; i<4; i++){
      RobotContainer.swerveDrive.driveOneModule(i, speed, angle[i], false);
    }
    double[] moduleVelocities = RobotContainer.swerveDrive.getAllModuleVelocity();
    //push velocities pulled from the drive motors
    SmartDashboard.putNumber("frontLeftVelocity", moduleVelocities[SwerveModNum.frontLeft.getNumber()]);
    SmartDashboard.putNumber("frontRightVelocity", moduleVelocities[SwerveModNum.frontRight.getNumber()]);
    SmartDashboard.putNumber("rearLeftVelocity", moduleVelocities[SwerveModNum.rearLeft.getNumber()]);
    SmartDashboard.putNumber("rearRightVelocity", moduleVelocities[SwerveModNum.rearRight.getNumber()]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
