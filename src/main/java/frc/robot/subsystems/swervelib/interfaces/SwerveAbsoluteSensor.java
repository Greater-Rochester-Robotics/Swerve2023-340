// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Swerve modules need an absolute sensor on modules to
 * detect the current position of the module. Here the 
 * base functions needed from the absolute sensor are 
 * defined.
 */
public interface SwerveAbsoluteSensor {

   /**
    * This method is used to change the offset of 
    * the absolute sensor so we dictate the zero 
    * position as the current position of the module.
    */
   public void zeroAbsPositionSensor();

   /**
    * The CANCoder reads the absolute rotational position
    * of the module. This method returns that positon in 
    * degrees.
    * note: NOT Inverted module safe (use getPosInRad())
    * 
    * @return the position of the module in degrees, should limit from -180 to 180
    */
   public double getAbsPosInDeg();

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad();

    /**
     * Returns the current angle of the swerve module, 
     * as read by the absolute rotational sensor, as a 
     * Rotation2d object. This is measured from the 
     * front of the robot, where counter-clockwise is 
     * positive.
     * 
     * @return A Rotation2d object, current position of the module
     */
    public Rotation2d getCurRot2d();

    /**
     * Returns the crrent speed the module is spinning 
     * in radians per second. Counter-clockwise should 
     * be positive.
     * 
     * @return a double of the speed.
     */
    public double getSpeedInRad();

  }
