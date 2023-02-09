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
 * to end.
 * 
 * UNLIKE DriveFieldCentric this command uses a PIDController 
 * to maintain the robot's rotational orientation when the 
 * robot is not instructed to rotate by the rotational 
 * input. 
 */

public class DriveFieldRelativeAdvanced extends CommandBase {
  private boolean isVeloMode;
  private double currentAngle = 0;
  private boolean wasDriverControl;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveFieldRelativeAdvanced(boolean isVeloMode) {
    addRequirements(RobotContainer.swerveDrive);
    this.isVeloMode = isVeloMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
    wasDriverControl = false;
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
    //create rotation speed from gamepad triggers
    double rotSpeed = Robot.robotContainer.getRobotRotation();

    //use DPad to turn to specific angles.
    // if(Robot.robotContainer.getDriverDPad() == 0){
    //   currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRad()/Constants.TWO_PI) * Constants.TWO_PI;
    // }
    // else if(Robot.robotContainer.getDriverDPad() == 90){
    //   currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRad()/Constants.TWO_PI) * Constants.TWO_PI - 1.178;
    // }

    //test if the absolute rotational input is greater than .1
    if (Math.abs(rotSpeed) > .1){
      //if the test is true, just copy the DriveFieldCentric execute method
      RobotContainer.swerveDrive.driveFieldRelative(
        awaySpeed,
        lateralSpeed,
        rotSpeed,
        isVeloMode
      );
      //for when rotation speed is zero, update the current angle
      currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
      //means that driver wants to turn so don't run counter rotation PID
      wasDriverControl = true;

    }
    else {
      if(wasDriverControl && Math.abs(RobotContainer.swerveDrive.getRotationalVelocityPitch()) > 90.0){
        RobotContainer.swerveDrive.driveFieldRelative(
          awaySpeed,
          lateralSpeed,
          0,
          isVeloMode
        );
        currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
      }else{
        //if the test is false, still use driveFieldCentric(), but for last parameter use PIDController accessor function
        RobotContainer.swerveDrive.driveFieldRelative(
          awaySpeed,
          lateralSpeed,
          RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
          isVeloMode
        );
        wasDriverControl = false;
      }
    }
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
