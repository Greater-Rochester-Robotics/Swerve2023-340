// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib.ctre;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swervelib.interfaces.SwerveAbsoluteSensor;

/** 
 * This class is the way to create a CANCoder object for use in the SwerveModule
 */
public class SwerveAbsoluteCANCoder implements SwerveAbsoluteSensor{
    CANCoder rotateAbsSensor;

    /**
     * 
     * @param canCoderID The CAN Device ID for the CANCoder
     */
    public SwerveAbsoluteCANCoder(int canCoderID){
        this( canCoderID, false);
    }

    /**
     * 
     * @param canCoderID The CAN Device ID for the CANCoder
     * @param isInverted If the CANCoder is inverted (placed underneath the module facing up)
     */
    public SwerveAbsoluteCANCoder(int canCoderID, boolean isInverted){
        //the following sensor is angle of the module, as an absolute value
        rotateAbsSensor = new CANCoder(canCoderID);

        //If the CANCoder is underneath the module, LED toward the floor, it must be inverted
        rotateAbsSensor.configSensorDirection(isInverted);

        //configure the cancoder to read values between -180 to 180
        rotateAbsSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        //The following reduces the CAN packets to a reasonable rate:
        //The following data packet has the default of 10, but 20 makes sense since the robot code loops at 20
        rotateAbsSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        //the following data packet is not needed in swerve
        rotateAbsSensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
    }
    

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to set the offset of the 
     * CANCoder so we can dictate the zero position. 
     * INPUTS MUST BE IN DEGREES. 
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    private void setRotateAbsSensor(double value) {
        rotateAbsSensor.configMagnetOffset(value, 0);
    }

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to change the offset of 
     * the CANCoder so we dictate the zero position as the 
     * current position of the module.
     */
    @Override
    public void zeroAbsPositionSensor() {
        setRotateAbsSensor(this.rotateAbsSensor.configGetMagnetOffset()-getAbsPosInDeg());
    }

    /**
     * The CANCoder reads the absolute rotational position
     * of the module. This method returns that positon in 
     * degrees.
     * 
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    @Override
    public double getAbsPosInDeg() {
        //The CANCOder reads values in degrees
        return rotateAbsSensor.getAbsolutePosition();
    }

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad() {
        //get the current position and convert it to radians.
        return Math.toRadians(getAbsPosInDeg());
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
     * Returns the speed that the module is rotating.
     * 
     * @return double speed of module in radians per second
     */
    public double getSpeedInRad(){
        return Math.toRadians(rotateAbsSensor.getVelocity());
    }
}
