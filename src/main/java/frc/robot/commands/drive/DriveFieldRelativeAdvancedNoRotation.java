// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interrupted 
 * to end. This command does not allow driver to rotate the robot with input.
 * 
 * UNLIKE DriveFieldCentric this command uses a PIDController 
 * to maintain the robot's rotational orientation
 */

public class DriveFieldRelativeAdvancedNoRotation extends CommandBase {
  private boolean isVeloMode;
  private double currentAngle = 0;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveFieldRelativeAdvancedNoRotation(boolean isVeloMode) {
    addRequirements(RobotContainer.swerveDrive);
    this.isVeloMode = isVeloMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //pull primary stick values, and put to awaySpeed and lateralSpeed doubles
    double awaySpeed = Robot.robotContainer.getRobotForwardFull(isVeloMode);
    double lateralSpeed = Robot.robotContainer.getRobotForwardFull(isVeloMode);
    //check if secondary sticks are being used
    if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
      //if secondary sticks used, replace with secondary sticks witha slow factor
      awaySpeed = Robot.robotContainer.getRobotForwardSlow(isVeloMode);
      lateralSpeed = Robot.robotContainer.getRobotLateralSlow(isVeloMode);
    }
  
    RobotContainer.swerveDrive.driveFieldRelative(
      awaySpeed*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR,
      lateralSpeed*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR,
      RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
      isVeloMode
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
