// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;

/**
 * This command is designed so that a driver can drive
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away
 * from the driver. If this is true, then left and right
 * on the stick will cause the robot to move to the
 * driver's left and right, respectively. This command
 * does not end of its own accord so it must be interrupted
 * to end.
 */

public class DriveFieldRelative extends CommandBase {
    /**
     * If velocity mode should be used.
     */
    private boolean isVeloMode;
     /**
     * If the gyro should be used.
     */
    private boolean counterRotationOn;
    /**
     * The robot's current angle.
     */
    private double currentAngle = 0;
    /**
     * If movement is from the driver's controls.
     */
    private boolean wasDriverControl = false;
    
    /**
     * The previous DPad value.
     */
    private int prevDPad;

    private Timer slowTimer;
    private boolean wasSlowStick;
    
    public DriveFieldRelative (boolean isVeloMode) {
        this(isVeloMode, true);
    }

    /**
     * Creates a new DriveFieldRelative command.
     * @param isVeloMode If velocity mode should be used.
     */
    public DriveFieldRelative (boolean isVeloMode, boolean counterRotationOn) {
        addRequirements(RobotContainer.swerveDrive);
        this.isVeloMode = isVeloMode;
        this.counterRotationOn = counterRotationOn;

        slowTimer = new Timer();
        wasSlowStick = false;
    }

    @Override
    public void initialize () {
        // Set the current angle.
        currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
        slowTimer.reset();
        slowTimer.start();
    }

    @Override
    public void execute () {
        // Fetch the driver's current analog inputs.
        double awaySpeed = Robot.robotContainer.getRobotForwardFull(isVeloMode);
        double lateralSpeed = Robot.robotContainer.getRobotLateralFull(isVeloMode);
        double rotSpeed = Robot.robotContainer.getRobotRotation(isVeloMode);
        boolean slowStick = Robot.robotContainer.getDriverButton(9);

        //check if secondary sticks are being used
        // if(slowStick){
        //     if(!wasSlowStick)
        //         slowTimer.reset();

        //     if(slowTimer.get() <= Constants.SwerveDriveConstants.DRIVER_SLOW_STICK_TIMEOUT || Constants.SwerveDriveConstants.DRIVER_SLOW_STICK_TIMEOUT <= 0) {
        //         //if secondary sticks used, replace with secondary sticks with a slow factor
        //         awaySpeed *= SwerveDriveConstants.DRIVER_SLOW_STICK_MODIFIER;
        //         lateralSpeed *= SwerveDriveConstants.DRIVER_SLOW_STICK_MODIFIER;
        //         rotSpeed *= SwerveDriveConstants.DRIVER_SLOW_STICK_ROT_MODIFIER;
        //     }

        //     wasSlowStick = slowStick;
        // }

        // Use the DPad to turn to specific angles.
        if (counterRotationOn && Robot.robotContainer.getDriverButton(5)) {
            // Face away if DPad up.
            currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRadYaw() / Constants.TWO_PI) * Constants.TWO_PI;
        } else if (counterRotationOn && Robot.robotContainer.getDriverDPad() == 180 && prevDPad != 180) {
            // Face towards if DPad down.
            currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRadYaw() / Constants.TWO_PI) * Constants.TWO_PI - Math.PI;
        }

        // test if the absolute rotational input is greater than .1
        if (!counterRotationOn || Math.abs(rotSpeed) > 0) {
            // if the test is true, just copy the DriveFieldCentric execute method
            RobotContainer.swerveDrive.driveFieldRelative(
                awaySpeed,
                lateralSpeed,
                rotSpeed,
                isVeloMode);
            // for when rotation speed is zero, update the current angle
            currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
            // means that driver wants to turn so don't run counter rotation PID
            wasDriverControl = true;

        } else {
            if (wasDriverControl
                && Math.abs(RobotContainer.swerveDrive.getRotationalVelocityYaw()) > 90.0) {
                RobotContainer.swerveDrive.driveFieldRelative(
                    awaySpeed,
                    lateralSpeed,
                    0,
                    isVeloMode);
                currentAngle = RobotContainer.swerveDrive.getGyroInRadYaw();
            } else {
                // if the test is false, still use driveFieldCentric(), but for last parameter use PIDController accessor function
                RobotContainer.swerveDrive.driveFieldRelative(
                    awaySpeed,
                    lateralSpeed,
                    RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle)
                        * (isVeloMode ? Constants.SwerveDriveConstants.MAX_ROBOT_ROT_VELOCITY : 1.0),
                    isVeloMode);
                wasDriverControl = false;
            }
        }
        prevDPad = Robot.robotContainer.getDriverDPad();
    }

    @Override
    public boolean isFinished () {
        // Run continuously.
        return false;
    }
}
