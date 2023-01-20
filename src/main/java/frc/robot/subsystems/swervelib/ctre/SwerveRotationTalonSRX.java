// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swervelib.interfaces.SwerveAbsoluteSensor;
import frc.robot.subsystems.swervelib.interfaces.SwerveRotationMotor;

/** Add your docs here. */
public class SwerveRotationTalonSRX implements SwerveRotationMotor , SwerveAbsoluteSensor{

    //TODO:Make a constructor needs a talonSrX id, a TalonSRX config object, a conversion factor(meters) 

    @Override
    public void setRotationMotorPIDF(double P, double I, double D, double F) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getRelEncCount() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRelEncSpeed() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void driveRotateMotor(double dutyCycle) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRotationMotorPosition(double output) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stopRotation() {
        // TODO Auto-generated method stub
        
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
