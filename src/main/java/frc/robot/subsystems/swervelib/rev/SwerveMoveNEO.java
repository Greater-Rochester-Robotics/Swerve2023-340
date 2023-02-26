// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.subsystems.swervelib.interfaces.SwerveMoveMotor;

/** Add your docs here. */
public class SwerveMoveNEO implements SwerveMoveMotor{
    private CANSparkMax driveMotor;
    private boolean areValuesUpdated = false;

    public SwerveMoveNEO(int driveMotorID){
        this(driveMotorID, new NEOConfig());
    }

    public SwerveMoveNEO(int driveMotorID, NEOConfig config) {
        this(driveMotorID, config, 0.0);
    }

    public SwerveMoveNEO(int driveMotorID, NEOConfig config, double encToMetersConvFactor){
        areValuesUpdated = false;
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        
        //adjust PIDF if changed
        if(driveMotor.getPIDController().getP() != config.pidfConfig.P){
            driveMotor.getPIDController().setP(config.pidfConfig.P);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getI() != config.pidfConfig.I){
            driveMotor.getPIDController().setI(config.pidfConfig.I);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getD() != config.pidfConfig.D){
            driveMotor.getPIDController().setD(config.pidfConfig.D);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getFF() != config.pidfConfig.FF){
            driveMotor.getPIDController().setFF(config.pidfConfig.FF);
            areValuesUpdated = true;
        }

        //confirm desired brake mode
        if((driveMotor.getIdleMode() == IdleMode.kBrake) != config.isBrakeMode){
            setDriveMotorBrake(config.isBrakeMode);
            areValuesUpdated = true;
        }
        //confirm if motor is inverted
        if(driveMotor.getInverted() != config.isInverted){
            driveMotor.setInverted(config.isInverted);// Set motor inverted(set to true)
            areValuesUpdated = true;
        }
        //confirm voltage compensation mode voltage
        if(driveMotor.getVoltageCompensationNominalVoltage() < config.maxVoltage-.01 
            || driveMotor.getVoltageCompensationNominalVoltage() > config.maxVoltage + .01){
            driveMotor.enableVoltageCompensation(config.maxVoltage);
            areValuesUpdated = true;
        }

        driveMotor.getEncoder().setPositionConversionFactor(encToMetersConvFactor);
        driveMotor.getEncoder().setVelocityConversionFactor(encToMetersConvFactor);
        
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); 
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 2000);
       
        //if values have changed burn NEO flash
        if(areValuesUpdated){
            driveMotor.burnFlash();
        }
    }


    public void setDriveDutyCycle(double dutyCycle){
        driveMotor.set(dutyCycle);
    }

    public void setDriveSpeed(double speed){
        driveMotor.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public double getDriveDistance(){
        return driveMotor.getEncoder().getPosition();
    }

    public void setDriveMotorBrake(boolean brakeOn){
        if(brakeOn){
            driveMotor.setIdleMode(IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public void resetDriveMotorEncoder(){
        driveMotor.getEncoder().setPosition(0.0);
    }
    
    public void setDriveMotorPIDF(double P, double I, double D, double F){
        driveMotor.getPIDController().setP(P);
        driveMotor.getPIDController().setI(I);
        driveMotor.getPIDController().setD(D);
        driveMotor.getPIDController().setFF(F);
    }

    public void enableVoltageCompensation(double maximumVoltage){
        driveMotor.enableVoltageCompensation(maximumVoltage);

    }

    @Override
    public void stopMotor() {
        driveMotor.stopMotor();       
    }

}
