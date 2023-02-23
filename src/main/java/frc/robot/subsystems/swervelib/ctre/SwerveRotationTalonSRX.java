// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swervelib.interfaces.SwerveAbsoluteSensor;
import frc.robot.subsystems.swervelib.interfaces.SwerveRotationMotor;

/** Add your docs here. */
public class SwerveRotationTalonSRX implements SwerveRotationMotor , SwerveAbsoluteSensor{
    private TalonSRX rotationMotor;
    private final double ENC_TO_RADS_CONV_FACTOR;
    
    public SwerveRotationTalonSRX(int rotationMotorID) {
        this(rotationMotorID, new TalonSRXConfiguration());
    }

    public SwerveRotationTalonSRX(int rotationMotorID, TalonSRXConfiguration config) {
        this(rotationMotorID, config, 0.0);
    }

    public SwerveRotationTalonSRX(int rotationMotorID, TalonSRXConfiguration config, double encToRadConvFactor) {
        rotationMotor = new TalonSRX(rotationMotorID);
        rotationMotor.configAllSettings(config);
        rotationMotor.configSelectedFeedbackCoefficient(encToRadConvFactor);
        ENC_TO_RADS_CONV_FACTOR = encToRadConvFactor;
    }

    @Override
    public void setRotationMotorPIDF(double P, double I, double D, double F) {
        rotationMotor.config_kP(0, P);
        rotationMotor.config_kD(0, D);
        rotationMotor.config_kI(0, I);
        rotationMotor.config_kF(0, F);
    }

    @Override
    public double getRelEncCount() {
        return rotationMotor.getSelectedSensorPosition()*ENC_TO_RADS_CONV_FACTOR;
    }

    @Override
    public double getRelEncSpeed() {
        return rotationMotor.getSelectedSensorVelocity()*ENC_TO_RADS_CONV_FACTOR*10;
        
    }

    @Override
    public void driveRotateMotor(double dutyCycle) {
        rotationMotor.set(TalonSRXControlMode.PercentOutput, dutyCycle);
    }

    @Override
    public void setRotationMotorPosition(double output) {
        rotationMotor.set(TalonSRXControlMode.Position, output/ENC_TO_RADS_CONV_FACTOR/10);
    }

    @Override
    public void stopRotation() {
        rotationMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public void zeroAbsPositionSensor() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getAbsPosInDeg() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getPosInRad() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Rotation2d getCurRot2d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getSpeedInRad() {
        // TODO Auto-generated method stub
        return 0;
    }

}
