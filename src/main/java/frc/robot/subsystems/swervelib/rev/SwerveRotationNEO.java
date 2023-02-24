// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.rev;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.subsystems.swervelib.interfaces.SwerveAbsoluteSensor;
import frc.robot.subsystems.swervelib.interfaces.SwerveRotationMotor;

/** Add your docs here. */
public class SwerveRotationNEO implements SwerveRotationMotor , SwerveAbsoluteSensor {
    private CANSparkMax rotationMotor;
    private AbsoluteEncoder absoluteEncoder;
    public final double ENC_TO_RAD_CONV_FACTOR;
    public final double ABS_ENC_TO_RAD_CONV_FACTOR;
    public double absOffsetAngle;

    public SwerveRotationNEO(int rotationMotorID, double encToRadConvFactor){
        this(rotationMotorID, encToRadConvFactor, new NEOConfig());
    }

    public SwerveRotationNEO(int rotationMotorID, double encToRadConvFactor, NEOConfig config){
        this(rotationMotorID, encToRadConvFactor, config, 0);
        
    }
    public SwerveRotationNEO(int rotationMotorID, double encToRadConvFactor, NEOConfig config, double absEncToRadConvFactor){
        //contruct and setup rotation NEO
        rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
        absoluteEncoder = rotationMotor.getAbsoluteEncoder(Type.kDutyCycle);

        
        ENC_TO_RAD_CONV_FACTOR = encToRadConvFactor;
                
        rotationMotor.getEncoder().setPositionConversionFactor(ENC_TO_RAD_CONV_FACTOR);
        rotationMotor.getEncoder().setVelocityConversionFactor(ENC_TO_RAD_CONV_FACTOR);


        ABS_ENC_TO_RAD_CONV_FACTOR = absEncToRadConvFactor; 

        absoluteEncoder.setPositionConversionFactor(ABS_ENC_TO_RAD_CONV_FACTOR);
        absoluteEncoder.setVelocityConversionFactor(ABS_ENC_TO_RAD_CONV_FACTOR);
        
        // absOffsetAngle = Preferences.getDouble("SwerveRotationNeoAbsOffset" + rotationMotor.getDeviceId(), 0.0);
        // absoluteEncoder.setZeroOffset(absOffsetAngle);
        
        // use the integrated sensor with the primary closed loop and timeout is 0.
        boolean areValuesUpdated = false;

        //adjust PIDF if changed
        if(rotationMotor.getPIDController().getP() != config.pidfConfig.P){
            rotationMotor.getPIDController().setP(config.pidfConfig.P);
            areValuesUpdated = true;
        }
        if(rotationMotor.getPIDController().getI() != config.pidfConfig.I){
            rotationMotor.getPIDController().setI(config.pidfConfig.I);
            areValuesUpdated = true;
        }
        if(rotationMotor.getPIDController().getD() != config.pidfConfig.D){
            rotationMotor.getPIDController().setD(config.pidfConfig.D);
            areValuesUpdated = true;
        }
        if(rotationMotor.getPIDController().getFF() != config.pidfConfig.FF){
            rotationMotor.getPIDController().setFF(config.pidfConfig.FF);
            areValuesUpdated = true;
        }

        //confirm desired brake mode
        if((rotationMotor.getIdleMode() == IdleMode.kBrake) != config.isBrakeMode){
            setRotationMotorBrake(config.isBrakeMode);
            areValuesUpdated = true;
        }
        //confirm if motor is inverted
        if(rotationMotor.getInverted() != config.isInverted){
            rotationMotor.setInverted(config.isInverted);// Set motor inverted(set to true)
            areValuesUpdated = true;
        }
        //confirm voltage compensation mode voltage
        if(rotationMotor.getVoltageCompensationNominalVoltage() < config.maxVoltage-.01 
            || rotationMotor.getVoltageCompensationNominalVoltage() > config.maxVoltage + .01){
            rotationMotor.enableVoltageCompensation(config.maxVoltage);
            areValuesUpdated = true;
        }

        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

        //if values have changed burn NEO flash
        if(areValuesUpdated){
            rotationMotor.burnFlash();
        }
    }

    public void setRotationMotorBrake(boolean brakeOn){
        if(brakeOn){
            rotationMotor.setIdleMode(IdleMode.kBrake);
        }
        else{
            rotationMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setRotationMotorPIDF(double P, double I, double D, double F){
        rotationMotor.getPIDController().setP(P);
        rotationMotor.getPIDController().setI(I);
        rotationMotor.getPIDController().setD(D);
        rotationMotor.getPIDController().setFF(F);
    }

    /**
     * 
     * @return get the relative encoder count in native units
     */
    public double getRelEncCount(){
        return rotationMotor.getEncoder().getPosition();
    }

    @Override
    public double getRelEncSpeed() {
        return rotationMotor.getEncoder().getVelocity();
    }

    /**
     * 
     */
    public void driveRotateMotor(double dutyCycle){
        this.rotationMotor.set(dutyCycle);
    }

    /**
     * 
     */
    public void setRotationMotorPosition(double output){
        rotationMotor.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
    }

    /**
     * A method to stop the rotation motor from rotating
     */
    public void stopRotation(){
        rotationMotor.stopMotor();
    }

    //external sensor 

    /**
     * This method is used to change the offset of 
     * the absolute sensor so we dictate the zero 
     * position as the current position of the module.
     */
    public void zeroAbsPositionSensor(){
        //set current position as zero, by making current sensor reading the offset
        absOffsetAngle = absoluteEncoder.getPosition() - absoluteEncoder.getZeroOffset();
        absoluteEncoder.setZeroOffset(absOffsetAngle);
        rotationMotor.burnFlash();
        //commit offset value to preferences table
        Preferences.setDouble("SwerveRotationNeoAbsOffset" + rotationMotor.getDeviceId(), absOffsetAngle);
    }

    /**
     * The CANCoder reads the absolute rotational position
     * of the module. This method returns that positon in 
     * degrees.
     * 
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getAbsPosInDeg(){
        return Math.toDegrees(this.getPosInRad());
    }

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad(){
        return absoluteEncoder.getPosition();
    }

    /**
     * Returns the current angle of the swerve module, 
     * as read by the absolute rotational sensor, as a 
     * Rotation2d object. This is measured from the 
     * front of the robot, where counter-clockwise is 
     * positive.
     * 
     * @return A Rotation2d object, current position of the module
     */
    public Rotation2d getCurRot2d(){
        return new Rotation2d(getPosInRad());
    }

    /**
     * This method gets the current velocity of the module in 
     * radians per second and is zero if the module is stopped.
     * 
     * @return the velocity of the module in radians per second
     */
    public double getSpeedInRad(){
        return absoluteEncoder.getVelocity();
    }

}
