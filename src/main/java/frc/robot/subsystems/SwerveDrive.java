// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.Constants;
import frc.robot.subsystems.swervelib.ADIS16470_IMU;
import frc.robot.subsystems.swervelib.SwerveModule;
import frc.robot.subsystems.swervelib.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.swervelib.rev.SwerveMoveNEO;
import frc.robot.subsystems.swervelib.rev.SwerveRotationNEO;


/**
 * This subsystem contains all SwerveModule objects, and runs all drive functions.
 * 
 * More on swerve found here:
 * https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing
 */
public class SwerveDrive extends SubsystemBase {
  /** Pose Estimator */
  PoseEstimator poseEst;

  /** Motors */
  private static SwerveMoveNEO swerveMoveNEO[];
  private static SwerveRotationNEO swerveRotationNEO[];
  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;

  /** Gyro */
  public ADIS16470_IMU imu;

  /** Kineatics and Odometry */
  private SwerveDriveKinematics driveKinematics;
  public SwerveGRROdometry driveOdometry;

  /** PID Controllers */
  private PIDController robotSpinController;
  private PIDController robotCounterSpinController;

  /** Trajectory */
  public PPHolonomicDriveController pathController;
  public PIDController xController;
  public PIDController yController;
  public PIDController rotationController;

  /** Booleans */
  private boolean hasPoseBeenSet = false;
  private boolean isOdometry = true;

  /**
   * This enumeration clarifies the numbering of the swerve module for new users.
   * frontLeft  | 0
   * rearLeft   | 1
   * rearRight  | 2
   * frontRight | 3
   */
  public enum SwerveModNum{
    frontLeft(0) , rearLeft(1) , rearRight(2) , frontRight(3);
    public int num;
    private SwerveModNum(int number){
      num = number;
    }
    public int getNumber() {
			return num;
		}
  }
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    //Sets the move NEO's configuration
    
    //Constructs all of the move motors
    swerveMoveNEO = new SwerveMoveNEO[]{
      new SwerveMoveNEO(Constants.FRONT_LEFT_MOVE_MOTOR, Constants.SwerveDriveConstants.MOVE_CONFIG, Constants.SwerveDriveConstants.DRIVE_ENC_TO_METERS_FACTOR), 
      new SwerveMoveNEO(Constants.REAR_LEFT_MOVE_MOTOR, Constants.SwerveDriveConstants.MOVE_CONFIG, Constants.SwerveDriveConstants.DRIVE_ENC_TO_METERS_FACTOR), 
      new SwerveMoveNEO(Constants.REAR_RIGHT_MOVE_MOTOR, Constants.SwerveDriveConstants.MOVE_CONFIG, Constants.SwerveDriveConstants.DRIVE_ENC_TO_METERS_FACTOR), 
      new SwerveMoveNEO(Constants.FRONT_RIGHT_MOVE_MOTOR, Constants.SwerveDriveConstants.MOVE_CONFIG, Constants.SwerveDriveConstants.DRIVE_ENC_TO_METERS_FACTOR )
    };

    //Constructs all of the rotation motors
    swerveRotationNEO = new SwerveRotationNEO[]{
      new SwerveRotationNEO(Constants.FRONT_LEFT_ROTATE_MOTOR, Constants.SwerveDriveConstants.ENC_TO_RAD_CONV_FACTOR, Constants.SwerveDriveConstants.ROTATE_CONFIG, Constants.TWO_PI),
      new SwerveRotationNEO(Constants.REAR_LEFT_ROTATE_MOTOR, Constants.SwerveDriveConstants.ENC_TO_RAD_CONV_FACTOR, Constants.SwerveDriveConstants.ROTATE_CONFIG, Constants.TWO_PI),
      new SwerveRotationNEO(Constants.REAR_RIGHT_ROTATE_MOTOR, Constants.SwerveDriveConstants.ENC_TO_RAD_CONV_FACTOR, Constants.SwerveDriveConstants.ROTATE_CONFIG, Constants.TWO_PI),
      new SwerveRotationNEO(Constants.FRONT_RIGHT_ROTATE_MOTOR, Constants.SwerveDriveConstants.ENC_TO_RAD_CONV_FACTOR, Constants.SwerveDriveConstants.ROTATE_CONFIG, Constants.TWO_PI)
    };

    // Constructs the swerve modules 
    frontLeft = new SwerveModule(swerveMoveNEO[0], swerveRotationNEO[0]);
    rearLeft = new SwerveModule(swerveMoveNEO[1], swerveRotationNEO[1]);
    rearRight = new SwerveModule(swerveMoveNEO[2], swerveRotationNEO[2]);
    frontRight = new SwerveModule(swerveMoveNEO[3], swerveRotationNEO[3]);
    
     //This may seem repetitive, but it makes clear which module is which.
    swerveModules = new SwerveModule[]{
      frontLeft,
      rearLeft,
      rearRight,
      frontRight
    };
    
    //Create kinematics object, which converts between ChassisSpeeds and ModuleStates
    driveKinematics = new SwerveDriveKinematics(
      Constants.SwerveDriveConstants.FRONT_LEFT_POSITION, Constants.SwerveDriveConstants.REAR_LEFT_POSITION, 
      Constants.SwerveDriveConstants.REAR_RIGHT_POSITION, Constants.SwerveDriveConstants.FRONT_RIGHT_POSITION);

    // Constructs IMU object (gyro)
    imu = new ADIS16470_IMU(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY);

    //construct the odometry class.
    driveOdometry = new SwerveGRROdometry(driveKinematics, getGyroRotation2d(), getSwerveModulePositions());

    //construct the wpilib PIDcontroller for rotation.

    robotSpinController = new PIDController(Constants.SwerveDriveConstants.ROBOT_SPIN_P, Constants.SwerveDriveConstants.ROBOT_SPIN_I, Constants.SwerveDriveConstants.ROBOT_SPIN_D);
    robotSpinController.setTolerance(Constants.SwerveDriveConstants.ROBOT_SPIN_PID_TOLERANCE);

    //construct the wpilib PIDcontroller for counter rotation.
    robotCounterSpinController = new PIDController(Constants.SwerveDriveConstants.ROBOT_COUNTER_SPIN_P, Constants.SwerveDriveConstants.ROBOT_COUNTER_SPIN_I, Constants.SwerveDriveConstants.ROBOT_COUNTER_SPIN_D);
    robotCounterSpinController.setTolerance(Constants.SwerveDriveConstants.ROBOT_SPIN_PID_TOLERANCE);

    //setup PID controllers for the xPostion on the field, the yPostion on the field, and the rotation of the robot
    xController = new PIDController(Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_D);
    yController = new PIDController(Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_D);
    rotationController = new PIDController(Constants.SwerveDriveConstants.DRIVE_ROTATION_CONTROLLER_P, Constants.SwerveDriveConstants.DRIVE_ROTATION_CONTROLLER_I, Constants.SwerveDriveConstants.DRIVE_ROTATION_CONTROLLER_D);

    rotationController.disableContinuousInput();//our gyro isn't discontinous

    //pass the three PID controllers into the one drive controller
    this.pathController = new PPHolonomicDriveController(xController, yController, rotationController);

    hasPoseBeenSet = false;
    isOdometry = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("GyroRoll", this.getGyroInDegRoll());
    SmartDashboard.putNumber("GyroPitch", this.getGyroInDegPitch());
    SmartDashboard.putNumber("GyroYaw", this.getGyroInDegYaw());

    SmartDashboard.putNumber("GyroRollVel", this.getRotationalVelocityRoll());
    SmartDashboard.putNumber("GyroPitchVel", this.getRotationalVelocityPitch());

    for(int i = 0; i < swerveModules.length; i++) {
      SmartDashboard.putNumber("Module relative encoder " + i, swerveModules[i].getRotationMotor().getRelEncCount());
    }

    //run odometry update on the odometry object
    driveOdometry.update(getGyroRotation2d(), getSwerveModulePositions(), getGyroInRadPitch(), getGyroInRadRoll());
    // SmartDashboard.putNumber("GyroRate", this.getRotationalVelocity());
    // SmartDashboard.putNumber("Odometry X", getCurPose2d().getX());
    // SmartDashboard.putNumber("Odometry Y", getCurPose2d().getY());
  }

  /**
   * Checks if odometry is being used
   * @return true is odometry is on
   */
  public boolean getIsOdometry(){
    return isOdometry;
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param chassisSpeeds an object  
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveRobotCentric(ChassisSpeeds chassisSpeeds , boolean isVeloMode, boolean rotationOnlyMode){
    //instantiate an array of SwerveModuleStates, set equal to the output of toSwerveModuleStates() 
    SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //use SwerveDriveKinematic.desaturateWheelSpeeds(), max speed is 1 if percentOutput, MaxVelovcity if velocity mode
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, isVeloMode? Constants.SwerveDriveConstants.PATH_MAXIMUM_VELOCITY : 1.0);
    
    for (int i = 0; i < targetStates.length; i++) {
      if(!rotationOnlyMode && Math.abs(targetStates[i].speedMetersPerSecond) < (isVeloMode?Constants.SwerveDriveConstants.MINIMUM_DRIVE_SPEED:Constants.SwerveDriveConstants.MINIMUM_DRIVE_DUTY_CYCLE)){
        //stop this module if speed is too slow
        swerveModules[i].stopAll();
      }else{
        //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
        swerveModules[i].setModuleState(targetStates[i], isVeloMode);
      }
    }
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param forwardSpeed the movement forward and backward
   * @param strafeSpeed the movement side to side
   * @param rotSpeed the speed of rotation
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed, boolean isVeloMode, boolean rotationOnlyMode){
    //convert forwardSpeed, strafeSpeed and rotSpeed to a chassisSpeeds object, pass to driveRobotCentric
    driveRobotCentric(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed), isVeloMode, rotationOnlyMode);
  }

  /**
   * Drive the robot so that all directions are independent of the robots orientation (rotation)
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * 
   * @param awaySpeed from field relative, aka a fix direction,
   *                  away from or toward the driver, a speed
   *                  valued between -1.0 and 1.0, where 1.0
   *                  is to away from the driver 
   * @param lateralSpeed from field relative, aka a fix direction
   *                     regardless of robot rotation, a speed
   *                     valued between -1.0 and 1.0, where 1.0
   *                     is to the left 
   * @param rotSpeed rotational speed of the robot
   *                 -1.0 to 1.0 where 0.0 is not rotating
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveFieldRelative(double awaySpeed, double lateralSpeed, double rotSpeed, boolean isVeloMode){
    //convert awaySpeed, lateralSpeed and rotSpeed to ChassisSpeeds with fromFieldRelativeSpeeds pass to driveRobotCentric
    driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(awaySpeed, lateralSpeed, rotSpeed, getGyroRotation2d()), isVeloMode, false);
  }

/**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber which of the four modules(0-3) we are using
   * @param moveSpeed move speed -1.0 to 1.0, where 0.0 is stopped
   * @param rotatePos a position between -PI and PI where we want the module to be
   * @param isVeloMode changes between velocity mode and dutyCycle mode
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos, boolean isVeloMode){
    //test that moduleNumber is between 0-3, return if not(return;)
    if (moduleNumber > 3 && moduleNumber < 0){
      System.out.println("Module " + moduleNumber + " is out of bounds.");
      return;
    }else if(rotatePos < -Math.PI || rotatePos > Math.PI){
      System.out.println("Input angle out of range.");
      return;
    }

    SwerveModuleState oneSwerveState = new SwerveModuleState(moveSpeed, new Rotation2d(rotatePos));
    //code to drive one module in a testing form
    swerveModules[moduleNumber].setModuleState( oneSwerveState , isVeloMode );
  }

  /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    //run a for loop to call each module
    for (SwerveModule module : swerveModules) {
      //use the stopAll method, which stops both the drive and rotation motor.
      module.stopAll();
    }
  }

  /**
   * Pull the current Position from the odometry 
   * class, this should be in meters.
   * 
   * @return a Pose2d representing the current position
   */
  public Pose2d getCurPose2d(){
    return driveOdometry.getPoseMeters();
  }

  /**
   * Sets current position in the odometry class
   * 
   * @param pose new current position
   */
  public void setCurPose2d(Pose2d pose) {

    driveOdometry.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), pose);
    hasPoseBeenSet = true;
  }

  /**
   * Returns whether or not the position of the robo on the 
   * field is known because Pose2D has been set
   * @return
   */
  public boolean hasPoseBeenSet() {
    return hasPoseBeenSet;
  }

  /**
   * this is left over 2022 stuff
   * @return angle in radians (not restricted to PI to -PI)
   */
  public double getAngleOfTarget(){
    //get the current angle    
    double currentAngle = getGyroInRadRoll();

    //if the pose hasn't been set return current angle.
    if(!hasPoseBeenSet){
      return currentAngle;
    }

    // Calculating current angle between -pi and pi
    double absoluteCurrentAngle = currentAngle%Constants.TWO_PI;
    if(absoluteCurrentAngle > Math.PI){
      absoluteCurrentAngle -= 2*(Math.PI);
    }
    else if(absoluteCurrentAngle < -Math.PI){
      absoluteCurrentAngle += 2 * Math.PI;
    }

    // Finds where the center of the field is with respect to the robot
    Translation2d target = driveOdometry.getPoseMeters().getTranslation().minus(Constants.SwerveDriveConstants.FIELD_CENTER);
    //based on that, find the angle of the above Tanslation2d object
    double desiredAngle = Math.atan2(target.getY(), target.getX());

    // Calculate the robot's target angle given the continuous angle of the gyroscope
    return currentAngle - absoluteCurrentAngle + desiredAngle;
  }


  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
   */
  public void resetGyro(){
    imu.resetAllAngles();
  }

  /**
   * A function that allows the user to set the gyro to a 
   * specific angle. This will make the current orientation 
   * of the robot the input value. This must be in degrees 
   * for gyro.
   * @param newCurrentAngle value the gyro should now read in degrees.
   */
  public void setGyroPitchAngle(double newCurrentAngle){
    imu.setGyroAngleY(newCurrentAngle);
  }

  public void setGyroRollAngle(double newCurrentAngle){
    imu.setGyroAngleX(newCurrentAngle);
  }

  public void setGyroYawAngle(double newCurrentAngle){
    imu.setGyroAngleZ(newCurrentAngle);
  }

  
  /**
   * This calls the drive Gyro and returns the Rotation2d object.
   * This object contains angle in radians, as well as the sin 
   * and cos of that angle. This is an object that represents the
   * rotation of the robot.
   * @return a Rotation2d object
   */
  public Rotation2d getGyroRotation2d(){
    //return a newly constructed Rotation2d object, it takes the angle in radians as a constructor argument
    return Rotation2d.fromDegrees(getGyroInDegYaw());
    //note that counterclockwise rotation is positive
  }

  // public double getGyroInRad(){
  //   return Math.toRadians(getGyroInDeg()); // Pull the gyro in degrees, convert and return in radians
  //   //note that counterclockwise rotation is positive
  // }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, should be positive when right
   * side of robot is tilted upwards
   * @return The angle of the robot roll in radians
   */
  public double getGyroInRadRoll(){
    return Math.toRadians(getGyroInDegRoll());
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, should be positive when front
   * of robot is tilted upwards
   * @return The angle of the robot pitch in radians
   */
  public double getGyroInRadPitch(){
    return Math.toRadians(getGyroInDegPitch());
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, should be positive when robot
   * turned left
   * @return The angle of the robot yaw in radians
   */
  public double getGyroInRadYaw(){
    return Math.toRadians(getGyroInDegYaw());
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, should be positive when right
   * side of robot is tilted upwards
   * @return The angle of the robot roll in degrees
   */
  public double getGyroInDegRoll(){
    return imu.getAngle(IMUAxis.kY);
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, should be positive when front
   * of robot is tilted upwards
   * @return The angle of the robot pitch in degrees
   */
  public double getGyroInDegPitch(){
    return imu.getAngle(IMUAxis.kX);
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, should be positive when robot
   * turned left
   * @return The angle of the robot yaw in radians
   */
  public double getGyroInDegYaw(){
    return imu.getAngle(IMUAxis.kZ);    
  }

  /**
   * Returns the speed of rotation of the robot 
   * roll, right side going up is positive.
   * @return degrees per second
   */
   public double getRotationalVelocityRoll(){
    return imu.getRate(IMUAxis.kY);
  }

  /**
   * Returns the speed of rotation of the robot 
   * pitch, front going up is positive.
   * @return degrees per second
   */
  public double getRotationalVelocityPitch(){
    return imu.getRate(IMUAxis.kX);
  }

  /**
   * Returns the speed of rotation of the robot 
   * yaw, turning left is positive
   * @return degrees per second
   */
  public double getRotationalVelocityYaw(){
    return imu.getRate(IMUAxis.kZ);
  }

  /**
   * Finds nearest angle that would line the front
   * of the robot up parallel with the ramp,
   * for use in auto balancing
   * @return nearest multiple of 90 degrees angle to gyro yaw
   */
  public double findNearestAngle() {
    double currentAngle = getGyroInDegYaw();
    double distance = currentAngle % 90;
    if(Math.abs(distance) > 45) {
      return currentAngle + (Math.signum(distance)*90 - distance);
    }else{
      return currentAngle - distance;
    }
  }

  /**
   * Accessor method for module positions array, puts all swerve
   * module positions in a new array and returns
   * @return new array with module positions
   */
  public SwerveModulePosition[] getSwerveModulePositions(){
    //instatiate and construct a 4 large SwerveModuleState array
    SwerveModulePosition[] modulePositions =  new SwerveModulePosition[4];
    //get the current SwerveModuleStates from all modules in array
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = swerveModules[i].getModulePosition();
    }

    return modulePositions;
  }

  /**
   * Accessor method for module states array, puts all swerve
   * module states in a new array and returns
   * @return new array with module states
   */
  public SwerveModuleState[] getSwerveModuleStates(){
    //instatiate and construct a 4 large SwerveModuleState array
    SwerveModuleState[] moduleStates =  new SwerveModuleState[4];
    //get the current SwerveModuleStates from all modules in array
    for (int i = 0; i < moduleStates.length; i++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }

    return moduleStates;
  }

  /**
   * Returns all values from the module's absolute 
   * encoders, and returns them in an array of 
   * doubles, as degrees, in module order.
   * 
   * @return array of doubles, in degrees
   */
  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getModulePosition().angle.getDegrees();
    }
    return moduleAngles;
  }

  /**
   * Returns all values from the rotational motor's 
   * relative encoders in an array of doubles. This 
   * array is in order of module number.
   * 
   * @return array of doubles, representing tick count.
   */
  public double[] getAllModuleRelEnc(){
    double[] moduleRelEnc = new double[4];
    for(int i=0; i<4; i++){
      moduleRelEnc[i]=swerveRotationNEO[i].getRelEncCount();
    }
    return moduleRelEnc;
  }

  /**
   * Returns the collective distance as seen by the 
   * drive motor's encoder, for each module.
   * 
   * @return an array of doubles in meters
   */
  public double[] getAllModuleDistance(){
    double[] moduleDistances = new double[4];
    for(int i=0; i<4; i++){
      moduleDistances[i]=swerveModules[i].getModulePosition().distanceMeters;
    }
    return moduleDistances;
  }

  /**
   *  Gets all the drive velocities.
   * 
   * @return An array of velocities.
   */
  public double[] getAllModuleVelocity(){
    double[] moduleVelocities = new double[4];
    for(int i=0; i<4; i++){
      moduleVelocities[i]=swerveModules[i].getModuleState().speedMetersPerSecond;
    }
    return moduleVelocities;
  }

  /**  
   * method to configure all modules DriveMotor PIDF
   * these are the PIDF on the TalonFX
   */
  public void setDrivePIDF(double P, double I, double D, double F){
    for(SwerveMoveNEO NEO : swerveMoveNEO) {
      NEO.setDriveMotorPIDF(P, I, D, F);
    }
  }


  /**
   * a method to print all module positions for testing purposes
   */
  public void printAllModuleAngles(){
    //Use a for loop to and print() all modules' angles(degrees) on one line  
    System.out.print("Angle = ");
    for (SwerveModule module : swerveModules) {
      System.out.print(module.getModulePosition().angle.getDegrees()+"\t");
    }
    //make sure to newline "\n" at the end
    System.out.print("\n");
  }
  
  /**
   * Method for taking the current position of all modules,
   * and making that position the absolute zero of each 
   * modules position respectively.
   */
  public void zeroAllModulePosSensors(){
    //a for loop so cycle through all modules
    for (SwerveModule module : swerveModules) {
      //call the zero position method
      module.zeroAbsPositionSensor();
    }
  }

  /**
   * Gets output of rotational PID given a target angle
   * @param target an angle in radians
   * @return a value to give the rotational input, -1.0 to 1.0
   */
  public double getRobotRotationPIDOut(double target){
    double currentGyroPos = getGyroInRadRoll();
    double output = robotSpinController.calculate(currentGyroPos, target);
    // System.out.println("targetAngle:"+Math.toDegrees(target)+"   angle:"+Math.toDegrees(currentGyroPos)+"atSP:"+robotSpinController.atSetpoint()+"  pid output"+output);
    if(robotSpinController.atSetpoint()){
      return 0.0;
    } else {
      if (Math.abs(output) < Constants.SwerveDriveConstants.MINIMUM_ROTATIONAL_OUTPUT){
        return Constants.SwerveDriveConstants.MINIMUM_ROTATIONAL_OUTPUT*Math.signum(output);
      }else {
        return output;
      }
    }
  }

  /**
   * Gets output of rotational PID given a target angle
   * @param target an angle in radians
   * @return a value to give the rotational input, -1.0 to 1.0
   */
  public double getCounterRotationPIDOut(double target){
    double currentGyroPos = getGyroInRadRoll();
    return robotCounterSpinController.calculate(currentGyroPos, target);
  }

  /** Reset the PID controllers, zero the I error, etc. */
  public void resetTrajectoryPIDControllers() {
    xController.reset();
    yController.reset();
    rotationController.reset();
  }

  /**
   * Using the desiredState and the currentState, use the pathController to find the speeds the robot should be going
   * @param desiredState {@link PathPlannerState} robot needs to be at
   * @return {@link ChassisSpeeds} motors should move at to reach desired state
   */
  public ChassisSpeeds calculateSpeedsTraj(PathPlannerState desiredState) {
    return pathController.calculate(getCurPose2d(), desiredState);
  }
}


