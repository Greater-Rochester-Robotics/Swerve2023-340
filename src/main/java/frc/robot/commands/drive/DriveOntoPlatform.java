// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOntoPlatform extends SequentialCommandGroup {
  /** Creates a new DriveOntoPlatform. */
  public DriveOntoPlatform() {
    addCommands(
      //Aligns robot to go straight onto platform
      new DriveTurnToAngleInRad(RobotContainer.swerveDrive::findNearestAngle),
      //Driver drive up platform with ability to move forward, back, right, and left but not rotate
      //Wait unil robot is tilted up and therefore far enough up the ramp
      Commands.deadline(
        new WaitUntilCommand(this::isTilted), 
        new DriveFieldRelativeAdvancedNoRotation(false)
      ),
      //Balance on ramp, driver maintains ability to move left and right but not forward and back
      //Stop when gyro says we are balanced to continue
      Commands.deadline(
        new WaitUntilCommand(this::isBalanced),
        new DriveBalanceRobot()
      ),
      //Lock wheels so we don't move
      new DriveLockWheels()
    );
    //note: could be going up on roll or pitch, platform is going to rotate, other robots rotating platform too
  }

  private boolean isTilted() {
    return Math.abs(RobotContainer.swerveDrive.getGyroInDegPitch()) > 10.0 || Math.abs(RobotContainer.swerveDrive.getGyroInDegRoll()) > 10.0; //TODO: put correct angle
  }

  private boolean isBalanced() {
    return
      (Math.abs(RobotContainer.swerveDrive.getGyroInDegPitch()) < 10.0 && Math.abs(RobotContainer.swerveDrive.getGyroInDegRoll()) < 10.0) && //TODO: put correct angle
      (Math.abs(RobotContainer.swerveDrive.getRotationalVelocityPitch()) < 10.0 || Math.abs(RobotContainer.swerveDrive.getRotationalVelocityRoll()) < 10.0); //TODO: put correct velocity
  }
}
