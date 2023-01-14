// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import frc.robot.subsystems.swervelib.interfaces.SwerveMoveBase;

/** Add your docs here. */
public class SwerveMoveTalonFX extends SwerveMoveBase {
    //TODO:finish writing this class
    private TalonFX driveMotor;

    public void setDriveDutyCycle(double dutyCycle){
        driveMotor.set(ControlMode.PercentOutput, dutyCycle);
    }

    public void setDriveSpeed(double speed){
        driveMotor.set(ControlMode.Velocity, speed);
    }

    /**
     * A method to set the drive motor to brake
     * @param brakeOn
     */
    public void setDriveMotorBrake(boolean brakeOn){
        driveMotor.setNeutralMode(brakeOn ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public double getDriveDistance(){
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     */
    public void resetDriveMotorEncoder(){

    }

    /**
     * Returns the speed of the drive wheel in Meters per second
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity();
    }

        /**
     * sets the drive motor's PIDF for the PIDF controller on the controller
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public void setDriveMotorPIDF(double P, double I, double D, double F){

    }

    public void enableVoltageCompensation(double maximumVoltage){
        driveMotor.configVoltageCompSaturation(maximumVoltage);
    }

    public void stopMotor(){
        driveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public double getMinimumDriveSpeed() {
        //TODO: Auto-generated method stub
        return 0;
    }

    public double getMinimumDutyCycle() {
        //TODO: Auto-generated method stub
        return 0;
    }

}
