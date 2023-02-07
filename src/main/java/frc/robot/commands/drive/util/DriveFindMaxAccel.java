// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

/**
 * a function to find the max accelleration of the robot
 */
public class DriveFindMaxAccel extends CommandBase {
  private Timer timer = new Timer();
  private double speed = .75;
  private double[] angle = new double[]{Math.toRadians(135), Math.toRadians(-135), Math.toRadians(-45), Math.toRadians(45)};
  /** Creates a new DriveFindMaxAccel. */
  public DriveFindMaxAccel(double speed) {
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //Stops all modules and starts the timer
    // RobotContainer.swerveDrive.resetCurrentPos();
    RobotContainer.swerveDrive.stopAllModules();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Drives the robot forword
    // RobotContainer.swerveDrive.driveFieldRelative(1.0, 0.0, 0.0, false);
    for (int i=0; i<4; i++){
      RobotContainer.swerveDrive.driveOneModule(i, speed, angle[i], false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  //TODO: Should this be uncommented
  public void end(boolean interrupted) {
    //calculates the max acceleration
    // double accel = RobotContainer.swerveDrive.getCurrentVelocity().getX() *
    //   RobotContainer.swerveDrive.getCurrentVelocity().getX() /
    //   2 / RobotContainer.swerveDrive.getCurrentPose().getX();
    timer.stop();
    // System.out.println("Acceleration = " + accel);
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;//RobotContainer.swerveDrive.getCurrentVelocity().getX() > 4.0;
  }
}
