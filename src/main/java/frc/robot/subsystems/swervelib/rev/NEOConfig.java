// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.rev;

import frc.robot.subsystems.swervelib.SwervePIDFConfig;

/** Add your docs here. */
public class NEOConfig {
    public boolean isInverted;
    public boolean isBrakeMode;
    public double maxVoltage;
    public SwervePIDFConfig pidfConfig;

    public NEOConfig(){
        this(new SwervePIDFConfig(), true);
    }

    public NEOConfig(boolean isInverted){
        this( new SwervePIDFConfig(), isInverted);
    }

    public NEOConfig(SwervePIDFConfig pidfConfig){
        this( pidfConfig, true);
    }

    public NEOConfig(SwervePIDFConfig pidfConfig,boolean isInverted){
        this( pidfConfig, isInverted, true);
    }

    public NEOConfig(SwervePIDFConfig pidfConfig, boolean isInverted, boolean isBrakeMode){
        this(pidfConfig, isInverted,isBrakeMode,12.0);
    }

    public NEOConfig(SwervePIDFConfig pidfConfig, boolean isInverted, boolean isBrakeMode, double maxVoltage){
        this.pidfConfig = pidfConfig;
        this.isInverted = isInverted;
        this.isBrakeMode = isBrakeMode;
        this.maxVoltage = maxVoltage;
    }

    public void setInverted(boolean isInverted){
        this.isInverted = isInverted;
    }

    public boolean getInverted(){
        return this.isInverted;
    }

    
    public void setBrakeMode(boolean isBrakeMode){
        this.isBrakeMode = isBrakeMode;
    }

    public boolean getBrakeMode(){
        return this.isBrakeMode;
    }

    public void setMaxVoltage(double maxVoltage){
        this.maxVoltage = maxVoltage;
    }

    public double getMaxVoltage(){
        return this.maxVoltage;
    }
}
