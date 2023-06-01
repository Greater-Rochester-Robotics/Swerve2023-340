// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Drive up the charge station until balanced.
 * Stops moving when robot is coming to a balance.
 * Driver has lateral control if specified.
 */
public class DriveBalanceRobot extends CommandBase {
    /**
     * If the driver should have lateral control.
     */
    private boolean enableLateralControl;
    /**
     * The robot's starting yaw.
     */
    private double startingYaw = 0;
    /**
     * The robot's previous pitch.
     */
    private double prevPitch = 0;
    /**
     * The robot's previous roll.
     */
    private double prevRoll = 0;
    /**
     * The previous time of the last execute.
     */
    private double prevTime = 0;
    /**
     * The balancing PID controller.
     */
    private PIDController balancePIDX;
    private PIDController balancePIDY;

    /**
     * Creates a new DriveBalance command.
     * Doesn't give the driver lateral control.
     */
    public DriveBalanceRobot() {
        this(false);
    }

    /**
     * Creates a new DriveBalance command.
     * @param enableLateralControl If the driver should have lateral control.
     */
    public DriveBalanceRobot (boolean enableLateralControl) {
        addRequirements(RobotContainer.swerveDrive);
        this.enableLateralControl = enableLateralControl;
        balancePIDX = new PIDController(0.25, 0.0, 0.0);
        balancePIDY = new PIDController(0.25, 0.0, 0.0);
    }

    @Override
    public void initialize () {
        // Set current values.
        startingYaw = RobotContainer.swerveDrive.getGyroInRadYaw();
        prevPitch = RobotContainer.swerveDrive.getGyroInDegPitch();
        prevRoll = RobotContainer.swerveDrive.getGyroInDegRoll();
        prevTime = Timer.getFPGATimestamp();

        // Reset the PID controller.
        balancePIDX.reset();
        balancePIDY.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute () {
        // Get the current pitch and roll.
        double currentPitch = RobotContainer.swerveDrive.getGyroInDegPitch();
        double currentRoll = RobotContainer.swerveDrive.getGyroInDegRoll();

        // Determine angular velocity.
        double time = Timer.getFPGATimestamp();
        double deltaTime = time - prevTime;
        double pitchVel = (currentPitch - prevPitch) / deltaTime;
        double rollVel = (currentRoll - prevRoll) / deltaTime;

        // Update previous values for the next execute.
        prevTime = time;
        prevPitch = currentPitch;
        prevRoll = currentRoll;

        // Get the current gyro position.
        Rotation2d currentGyro = RobotContainer.swerveDrive.getGyroRotation2d();

        // if(enableLateralControl){
        // Determine the away speed.
        double awaySpeed = 0.0;
        double awayAngle = (currentPitch * currentGyro.getCos()) + (currentRoll * currentGyro.getSin());
        double awayVel = Math.sqrt((pitchVel * pitchVel) + (rollVel * rollVel));

        // If the charge station is moving down or the robot's pitch / roll is within tolerance, stop moving forward.
        if (Math.abs(awayVel) > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE || 
            Math.abs(awayAngle) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE) {
            awaySpeed = 0.0;
        } else {
            // Determine the speed the robot should move at.
            double rawSpeed = -balancePIDX.calculate(currentPitch, 0.0);

            // Set the away speed to the determined speed within the range of the specified max speed.
            awaySpeed = Math.min(Math.max(rawSpeed, -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED), SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED);
        }

        // Get the lateral speed from the driver's controller.
        double lateralSpeed = enableLateralControl ? Robot.robotContainer.getRobotLateralFull(false) : 0;
        if(Robot.robotContainer.getDriverButton(9)){
            //if secondary sticks used, replace with secondary sticks with a slow factor
            lateralSpeed *= .5;
        }

        // Move the robot.
        RobotContainer.swerveDrive.driveFieldRelative(
            awaySpeed,
            lateralSpeed, 
            RobotContainer.swerveDrive.getCounterRotationPIDOut(startingYaw), 
            false
        );
        // }else{
        //     double forwardSpeed = 0.0;
        //     double strafeSpeed = 0.0;
        //     //if we are facing up and the ramp is moving down (or vice versa) we are coming to balance so stop moving
        //     if(Math.abs(pitchVel) > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE || 
        //         Math.abs(currentPitch) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE){
                
        //         forwardSpeed = 0.0;
        //     } else {
        //         forwardSpeed = balancePIDX.calculate(currentPitch, 0.0);
        //     }

        //     //puts the value of forward speed between maxSpeed and -maxSpeed
        //     if(forwardSpeed > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
        //         forwardSpeed = SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
        //     }else if(forwardSpeed < -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
        //         forwardSpeed = -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
        //     }

        //     //same thing for roll
        //     // if(Math.abs(rollVel) > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE || 
        //     //     Math.abs(currentRoll) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE){
        //     // strafeSpeed = 0.0;
        //     // } else {
        //     // strafeSpeed = balancePIDY.calculate(currentRoll, 0.0);
        //     // }

        //     // //puts the value of strafe speed between maxSpeed and -maxSpeed
        //     // if(strafeSpeed > SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
        //     //     strafeSpeed = SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
        //     // }else if(strafeSpeed < -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED){
        //     //     strafeSpeed = -SwerveDriveConstants.DRIVE_BALANCE_ROBOT_MAX_SPEED;
        //     // }
        //     //moves the robot using driveRobotCentric
        //     RobotContainer.swerveDrive.driveRobotCentric(
        //         forwardSpeed * -1.0,
        //         strafeSpeed * -1.0,
        //         // RobotContainer.swerveDrive.getCounterRotationPIDOut(currentRoll),
        //         0,
        //         false,
        //         false
        //     );
        // }
    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the swerve modules.
        RobotContainer.swerveDrive.stopAllModules();
    }

    @Override
    public boolean isFinished () {
        // Run continuously.
        return false;
    }
}