// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.RobotContainer;

/**
 * Drive up ramp until balanced
 * relies upon pitch and roll angles and velocities
 * Stops moving when robot is coming to a balance
 * driver only maintains ability to move left and right
 */

public class DriveBalanceRobot extends CommandBase {
  private double currentAngle = 0;
  private PIDController pidX;
  private PIDController pidY;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveBalanceRobot() {
    addRequirements(RobotContainer.swerveDrive);
    //Instantiats and sets the tolerence for both pid controllers
    pidX = new PIDController(0.5, 0.0, 0.05); //TODO: Tune all pid and tolerance values
    pidX.setTolerance(0.05);
    pidY = new PIDController(0.5, 0.0, 0.05);
    pidY.setTolerance(0.05);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // RobotContainer.swerveDrive.setIsOdometry(false);
    currentAngle = RobotContainer.swerveDrive.getGyroInDegYaw();
    pidX.reset();
    pidY.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Gets the pitch yaw and their velocity's
    double pitchAng = RobotContainer.swerveDrive.getGyroInDegPitch();
    double rollAng = RobotContainer.swerveDrive.getGyroInDegRoll();
    double pitchVel = RobotContainer.swerveDrive.getRotationalVelocityPitch();
    double rollVel = RobotContainer.swerveDrive.getRotationalVelocityRoll();
    double forwardSpeed, strafeSpeed;

    //if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
    if(pitchAng * pitchVel < 0) {
      forwardSpeed = 0.0;
    } else {
      forwardSpeed = pidX.calculate(pitchAng, 0.0);
    }

    if(rollAng * rollVel < 0) {
      strafeSpeed = 0.0;
    } else {
      strafeSpeed = pidY.calculate(rollAng, 0.0);
    }
  
    // TODO: allow driver to move side to side
    //moves the robot using driveRobotCentric
    RobotContainer.swerveDrive.driveRobotCentric(
      forwardSpeed * -1.0,
      strafeSpeed * -1.0,
      RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
      false,
      false
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