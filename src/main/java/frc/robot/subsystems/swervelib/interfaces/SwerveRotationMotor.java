// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.interfaces;

/** Add your docs here. */
public interface SwerveRotationMotor {
    

    /**
     * sets the rotation motor's PIDF for the PIDF controller
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setRotationMotorPIDF(double P, double I, double D, double F);

    /**
     * This is a method meant for testing by getting the count from the 
     * rotational encoder which is internal to the NEO550. This encoder 
     * is relative, and does not easily translate to a specific rotational 
     * position of the swerve module.
     * 
     * @return the encoder count(no units, naturally just the count)
     */
    public double getRelEncCount();

    public double getRelEncSpeed();

    /**
     * This is a testing method, used to drive the module's rotation.
     * It takes pure motor duty cycle(percent output). Positive input 
     * should result in counter-clockwise rotation as viewed from the 
     * top. If not, the motor output must be inverted.
     * 
     * @param dutyCycle a percent output from -1.0 to 1.0, where 0.0 is stopped
     */
    public void driveRotateMotor(double dutyCycle);

    /**
     * set the rotation motor to a position based on the motors internal units.
     * @param output
     */
    public void setRotationMotorPosition(double output);

    /**
     * This method is used to stop the module rotation completely. 
     * The rotation motor's PIDController is set to DutyCyclevoltage 
     * control mode, and output of 0.0% output.
     */
    public void stopRotation();
    
}
