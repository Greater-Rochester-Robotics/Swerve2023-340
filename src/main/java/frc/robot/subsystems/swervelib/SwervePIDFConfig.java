// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

/** 
 * A class to hold PIDF settings for drive or rotation motors
 * 
 */
public class SwervePIDFConfig {
    public double P, I, D, FF, IZone, IMaxAccum;

    public SwervePIDFConfig(){}

    public SwervePIDFConfig(double P, double I, double D){
        this(P,I,D,0.0);
    }

    public SwervePIDFConfig(double P, double I, double D, double FF){
        this(P,I,D,FF,0.0,0.0);
    }

    public SwervePIDFConfig(double P, double I, double D, double FF, double IZone, double IMaxAccum){
        this.setP(P);
        this.setI(I);
        this.setD(D);
        this.setFF(FF);
        this.setIZone(IZone);
        this.setIMaxAccum(IMaxAccum);
    }

    /**
     * Set the proportional constant
     * @param P
     */
    public void setP(double P){
        this.P = P;
    }

    public double getP(){
        return this.P;
    }

    /**
     * Set the integral constant
     * @param I
     */
    public void setI(double I){
        this.I = I;
    }

    public double getI(){
        return this.I;
    }

    /**
     * Set the derivative constant
     * @param D
     */
    public void setD(double D){
        this.D = D;
    }

    public double getD(){
        return this.D;
    }

    /**
     * Set the feed forward constant
     * @param FF
     */
    public void setFF(double FF){
        this.FF = FF;
    }

    public double getFF(){
        return this.FF;
    }

    public void setIZone(double IZone){
        this.IZone = IZone;
    }

    public double getIZone(){
        return this.IZone;
    }
    
    public void setIMaxAccum(double IMaxAccum){
        this.IMaxAccum = IMaxAccum;
    }

    public double getIMaxAccum(){
        return this.IMaxAccum;
    }
}
