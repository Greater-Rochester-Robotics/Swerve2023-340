// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * Turns all wheels to form a circle
 */
public class DriveLockWheels extends CommandBase {
  
  public DriveLockWheels() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Sets all the wheels to e at 45 degree angles to the robots center
    RobotContainer.swerveDrive.driveOneModule(0, 0, 3.0 * Math.PI / 4.0,false);
    RobotContainer.swerveDrive.driveOneModule(1, 0, Math.PI / 4.0,false);
    RobotContainer.swerveDrive.driveOneModule(2, 0, 3.0 * Math.PI / 4.0,false);
    RobotContainer.swerveDrive.driveOneModule(3, 0, Math.PI / 4.0,false);
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
