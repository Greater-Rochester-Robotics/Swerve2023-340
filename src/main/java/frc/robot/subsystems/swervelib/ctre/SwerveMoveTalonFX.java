// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.subsystems.swervelib.interfaces.SwerveMoveMotor;

/** Add your docs here. */
public class SwerveMoveTalonFX implements SwerveMoveMotor {
    private TalonFX driveMotor;
    private final double ENC_TO_METERS_CONV_FACTOR;
    public SwerveMoveTalonFX(int driveMotorID) {
        this(driveMotorID, new TalonFXConfiguration());
    }

    public SwerveMoveTalonFX(int driveMotorID, TalonFXConfiguration config) {
        this(driveMotorID, config, 0.0);
    }

    public SwerveMoveTalonFX(int driveMotorID, TalonFXConfiguration config, double encToMetersConvFactor) {
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.configAllSettings(config);
        driveMotor.configSelectedFeedbackCoefficient(encToMetersConvFactor);
        ENC_TO_METERS_CONV_FACTOR = encToMetersConvFactor;
    }
    
    public void setDriveDutyCycle(double dutyCycle){
        driveMotor.set(ControlMode.PercentOutput, dutyCycle);
    }

    public void setDriveSpeed(double speed){
        driveMotor.set(ControlMode.Velocity, speed/ENC_TO_METERS_CONV_FACTOR/10);
    }

    /**
     * A method to set the drive motor to brake
     * @param brakeOn
     */
    public void setDriveMotorBrake(boolean brakeOn){
        driveMotor.setNeutralMode(brakeOn ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public double getDriveDistance(){
        return driveMotor.getSelectedSensorPosition()*ENC_TO_METERS_CONV_FACTOR;
    }

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     */
    public void resetDriveMotorEncoder(){
        driveMotor.setSelectedSensorPosition(0.0);
    }

    /**
     * Returns the speed of the drive wheel in Meters per second
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity()*ENC_TO_METERS_CONV_FACTOR*10;
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
        driveMotor.config_kP(0, P);
        driveMotor.config_kD(0, D);
        driveMotor.config_kI(0, I);
        driveMotor.config_kF(0, F);
    }

    public void enableVoltageCompensation(double maximumVoltage){
        driveMotor.configVoltageCompSaturation(maximumVoltage);
    }

    public void stopMotor(){
        driveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

}
