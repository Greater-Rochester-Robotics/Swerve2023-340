// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOntoPlatform extends SequentialCommandGroup {
  /** Creates a new DriveOntoPlatform. */
  public DriveOntoPlatform() {
    /*Aligns robot to go straight onto platform*/
    addCommands(new DriveTurnToAngleInRad(RobotContainer.swerveDrive::findNearestAngle));
    //Driver drive up platform with ability to move forward, back, right, and left but not rotate
    //Wait unil robot is tilted up and therefore far enogh up the ramp
    //Balance on ramp, driver maintains bility to move left and right but not forward and back
    //Stop when gyro says we are balanced and lock wheels

    //note: could be going up on roll or pitch, platform is going to rotate, other robots rotating platform too
  }
}
