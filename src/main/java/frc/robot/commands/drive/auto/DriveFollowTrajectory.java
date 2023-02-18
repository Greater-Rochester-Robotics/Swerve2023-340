// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * This command takes a file name as an arguement. It will 
 * then attempt to run said path by generating a trajectory 
 * from the path parameters in said file. PID values are 
 * from the Constants file. 
 */
public class DriveFollowTrajectory extends CommandBase {
  //timer for the running of the path
  Timer timer;

  //space for the trajectory, generated from the pathfile
  PathPlannerTrajectory trajectory;

  //whether of not this command resets the odometry to the starting point
  boolean resetOdometry;

  /**
   * Run the path with default MaxVelocity, MaxAcceleration
   * Did you remember to set the gyro at the start of the 
   * autonmomous to mathc the starting point found in here
   * 
   * @param pathFileName name of the file generated by pathplanner
   */
  public DriveFollowTrajectory(String pathFileName) {
    this(pathFileName, Constants.SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, Constants.SwerveDriveConstants.MAXIMUM_ACCELERATION);
  }

  public DriveFollowTrajectory(String pathFileName, double maxVel, double maxAccel) {
    this(pathFileName, maxVel, maxAccel, true);
  }

  public DriveFollowTrajectory(String pathFileName, double maxVel, double maxAccel, boolean resetOdometry) {
    //generate a trajectory from the path in the given file given the filename, max velocity and max acceleration.
    this(PathPlanner.loadPath(pathFileName, maxVel, maxAccel), resetOdometry);
  }

  public DriveFollowTrajectory(PathPlannerTrajectory trajectory) {
    this(trajectory, true);
  }
  
  public DriveFollowTrajectory(PathPlannerTrajectory trajectory, boolean resetOdometry) {
    //require the swerveDrive subsystem
    addRequirements(RobotContainer.swerveDrive);

    //construct the timer
    this.timer = new Timer();

    //generate a trajectory from the path in the given file given the filename, max velocity and max acceleration.
    this.trajectory = trajectory;

    //pass this value out of constructor
    this.resetOdometry = resetOdometry;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //start the timer, this must be reset also
    timer.reset();
    timer.start();

    //poll the trajectory to find the first point
    // PathPlannerState initialState = (PathPlannerState) trajectory.sample(0.0);
    PathPlannerState initialState = trajectory.getInitialState();

    //reset the PID controllers, zero the I error, etc.
    RobotContainer.swerveDrive.resetTrajectoryPIDControllers();

    //if we need to reset odometry...
    if(resetOdometry) {
      //set the current position to the expected pose fromt he trajectory
      RobotContainer.swerveDrive.setCurPose2d(new Pose2d(initialState.poseMeters.getTranslation(),RobotContainer.swerveDrive.getGyroRotation2d()));
      // RobotContainer.swerveDrive.setGyro(initialState.holonomicRotation.getDegrees());//safer to use the DriveSetGyro command
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //poll the current time
    double time = timer.get();

    //based on the current time, find where the trajectory says the robot should be
    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);

    ChassisSpeeds robotSpeed = RobotContainer.swerveDrive.calculateSpeedsTraj(desiredState);

    //pass the robotSpeed to the swerveDrive
    RobotContainer.swerveDrive.driveRobotCentric(robotSpeed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();

    //stop the robot as we are done with the path.
    RobotContainer.swerveDrive.driveRobotCentric(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when we have finished the time the path takes, end the command
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
