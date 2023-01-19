// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.interfaces;

/** 
 * This is an abstract class for the Drive motor controller for a 
 * swerve module. This motor is expected to drive the module wheel 
 * forward and backward. 
 */
public interface SwerveMoveMotor {

    /**
     * Set the speed of the drive motor in percent duty cycle
     * 
     * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
     *                  percent duty cycle
     */
    public abstract void setDriveDutyCycle(double dutyCycle);

    /**
     * Set the speed of the drive motor in meter per second.
     * 
     * @param speed a speed in meters per second
     */
    public abstract void setDriveSpeed(double speed); 

    /**
     * A method to set the drive motor to brake mode or to neutral mode.
     * @param brakeOn true if brake should be on, false if brake should be off 
     */
    public abstract void setDriveMotorBrake(boolean brakeOn);

    /**
     * @return the distance the drive wheel has traveled
     */
    public abstract double getDriveDistance();

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     * 
     */
    public abstract void resetDriveMotorEncoder();

    /**
     * Returns the speed of the drive wheel in meters per second
     * 
     * @return speed of the drive wheel
     */
    public abstract double getDriveVelocity();

    /**
     * Sets the drive motor's PIDF for the PIDF controller on the controller
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public abstract void setDriveMotorPIDF(double P, double I, double D, double F);

    /**
     * Set the drive motor to voltage compensation mode, 
     * where the motor attempts to adjust for changes in 
     * input voltage from the battery.
     * 
     * @param maximumVoltage
     */
    public abstract void enableVoltageCompensation(double maximumVoltage);
    
    /**
     * Method that stops the drive motor
     */
    public abstract void stopMotor();

}
