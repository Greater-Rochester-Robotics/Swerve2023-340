// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTurnToAngleInRad extends CommandBase {
  private double angle = 0;
  private DoubleSupplier angleSupplier;
  private int onTargetCount;
  private boolean isDoubleSupplierMode = false;
  private boolean angleTooBig = false;

  public DriveTurnToAngleInRad(DoubleSupplier angleSupplier) {
    isDoubleSupplierMode = true;
    addRequirements(RobotContainer.swerveDrive);
    this.angleSupplier = angleSupplier;
  }

  /** Creates a new DriveTurnToAngle. This command 
   * is used in tuning PID for rotation and in 
   * autonomous to make a specific turn. 
   * 
   * @param angle angle in radians 
   */
  public DriveTurnToAngleInRad(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);

    //Checks if the angle is in radians
    if(Math.abs(angle) > Constants.TWO_PI){
      angle = 0.0;
      angleTooBig = true;
    }
    this.angle = angle;
    isDoubleSupplierMode = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Gets the supplier if there is one
    if(isDoubleSupplierMode) {
      this.angle = angleSupplier.getAsDouble();
    }
    onTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = RobotContainer.swerveDrive.getRobotRotationPIDOut(angle);
    // System.out.println("pid output"+output);
    //Turns the robot
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, output, false, true);
    //If the robot is facing the target within a certain tolerence than it increses the onTargetCount if not it sets the onTargetCount to 0
    if(Math.abs(angle - RobotContainer.swerveDrive.getGyroInRadYaw()) < .03){
      onTargetCount++;
    }else{
      onTargetCount = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(angleTooBig){
      DriverStation.reportError("THE ANGLE USED IN THIS TurnToAngleInRad IS TOO BIG, LIKELY BECAUSE IT IS IN DEGREES AND NOT RADIANS", true);
    }
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCount >= 10; //The command is finished if it has stayed on target ten or more times 
  }
}
