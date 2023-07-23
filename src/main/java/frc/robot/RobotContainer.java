// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;



import frc.robot.commands.drive.DriveFieldRelative;
import frc.robot.commands.drive.DriveLockWheels;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveStopAllModules;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveAdjustModulesManually;
import frc.robot.commands.drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.drive.util.DriveFindMaxAccel;
import frc.robot.commands.drive.util.DriveOneModule;
import frc.robot.commands.drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.drive.util.DriveResetGyroToZero;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.drive.util.DriveTuneDriveMotorFeedForward;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;


import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's gamepads are defined here...
  
  static final XboxController driver = new XboxController(0);
  static final XboxController coDriver = new XboxController(1);

  ////////////////////
  // DRIVER BUTTONS //
  ////////////////////

  static final Trigger driverA = new JoystickButton(driver, 1);
  static final Trigger driverB = new JoystickButton(driver, 2);
  static final Trigger driverX = new JoystickButton(driver, 3);
  static final Trigger driverY = new JoystickButton(driver, 4);
  static final Trigger driverLB = new JoystickButton(driver, 5);
  static final Trigger driverRB = new JoystickButton(driver, 6);
  static final Trigger driverBack = new JoystickButton(driver, 7);
  static final Trigger driverStart = new JoystickButton(driver, 8);
  static final Trigger driverLS = new JoystickButton(driver, 9);
  static final Trigger driverRS = new JoystickButton(driver, 10);
  static final Trigger driverDUp = new POVButton(driver, 0);
  static final Trigger driverDDown = new POVButton(driver, 180);
  static final Trigger driverDLeft = new POVButton(driver, 270);
  static final Trigger driverDRight = new POVButton(driver, 90);
  // final Trigger driverLTButton = new JoyTriggerButton(driver, .3, Axis.LEFT_TRIGGER);//This is used in driving, don't enable
  // final Trigger driverRTButton = new JoyTriggerButton(driver, .3, Axis.RIGHT_TRIGGER);//This is used in driving, don't enable

  ///////////////////////
  // CO-DRIVER BUTTONS //
  ///////////////////////

  static final Trigger coDriverA = new JoystickButton(coDriver, 1);
  static final Trigger coDriverB = new JoystickButton(coDriver, 2);
  static final Trigger coDriverX = new JoystickButton(coDriver, 3);
  static final Trigger coDriverY = new JoystickButton(coDriver, 4);
  static final Trigger coDriverLB = new JoystickButton(coDriver, 5);
  static final Trigger coDriverRB = new JoystickButton(coDriver, 6);
  static final Trigger coDriverBack = new JoystickButton(coDriver, 7);
  static final Trigger coDriverStart = new JoystickButton(coDriver, 8);
  static final Trigger coDriverLS = new JoystickButton(coDriver, 9);
  static final Trigger coDriverRS = new JoystickButton(coDriver, 10);
  static final Trigger coDriverDUp = new POVButton(coDriver, 0);
  static final Trigger coDriverDDown = new POVButton(coDriver, 180);
  static final Trigger coDriverDLeft = new POVButton(coDriver, 270);
  static final Trigger coDriverDRight = new POVButton(coDriver, 90);
  static final Trigger coDriverLTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kLeftTrigger);
  static final Trigger coDriverRTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kRightTrigger);
  static final Trigger coDriverLTButton25 = new JoyTriggerButton(coDriver, .25, Axis.kLeftTrigger);
  static final Trigger coDriverRTButton25 = new JoyTriggerButton(coDriver, .25, Axis.kRightTrigger);

  //Climber next step button is aliased here.
  public static final Trigger climberManButton = coDriverA;
  public static final Trigger climberAutoButton = coDriverB;
  
  
  //The robot's subsystems are instantiated here

  public static SwerveDrive swerveDrive;
  
  //The sendable chooser for autonomous is constructed here
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //create(construct) subsystems
    
    swerveDrive = new SwerveDrive();
    // swerveDrive.setDefaultCommand(new DriveFieldRelativeAdvanced(false));
    

    //Add all autos to the auto selector
    configureAutoModes();

    // Configure the button bindings
    configureButtonBindings();

    //add some commands to dashboard for testing/configuring
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());//For setup of swerve
    SmartDashboard.putData(new DriveAdjustModulesManually());//For setup of swerve
    SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));//For setup of swerve
    SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));//For setup of swerve
    SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));//For setup of swerve
    SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));//For setup of swerve
    SmartDashboard.putData(new DriveAllModulesPositionOnly());
    SmartDashboard.putData(new DriveStopAllModules());//For setup of swerve
    SmartDashboard.putData("Lock Wheels", new DriveLockWheels());



    SmartDashboard.putNumber("SpeedIShoot",0.0);
    SmartDashboard.putNumber("angleIShoot",0.0);

    // SmartDashboard.putData(new AutoRightFiveBall());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* ==================== DRIVER BUTTONS ==================== */
    driverLB.whenActive(new DriveResetGyroToZero());
    // driverRB.whileActiveContinuous(new DriveOnTarget(0));
    driverBack.or(driverStart).toggleWhenActive(new DriveRobotCentric(false)); 
    // driverStart.whenPressed(new AutoMidFourBall());
    // driverStart.whileHeld(new SequentialCommandGroup(
    //   new DriveTurnToAngle(Math.toRadians(-135)).withTimeout(2.5),
    //   new WaitCommand(.5),
    //   new DriveFollowTrajectory("DriveLeftToOppBallShoot")
    // ));

    /* =================== CODRIVER BUTTONS =================== */
    
    
  }


  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   * They will appear in the order entered
   */
  private void configureAutoModes() {
    
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));
   
    // autoChooser.addOption("5 ball", new AutoRightFiveBall());


    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  
  /**
   * A method to return the value of a driver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1)
   * @param axis
   * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
   */
  public double getDriverAxis(Axis axis) {
    return (driver.getRawAxis(axis.value) < -.1 || driver.getRawAxis(axis.value) > .1)
        ? driver.getRawAxis(axis.value)
        : 0.0;
  }

  /**
   * Accessor method to set driver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setDriverRumble(double leftRumble, double rightRumble) {
    driver.setRumble(RumbleType.kLeftRumble, leftRumble);
    driver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
     * accessor to get the true/false of the buttonNum 
     * on the driver control
     * @param buttonNum
     * @return the value of the button
     */
    public boolean getDriverButton(int buttonNum) {
      return driver.getRawButton(buttonNum);
  }

  /**
   * Returns the int position of the DPad/POVhat based
   * on the following table:
   *    input    |return
   * not pressed |  -1
   *     up      |   0
   *   up right  |  45
   *    right    |  90
   *  down right | 135
   *    down     | 180
   *  down left  | 225
   *    left     | 270
   *   up left   | 315
   * @return
   */
  public int getDriverDPad() {
    return (driver.getPOV());
  }

  /**
   * A method to return the value of a codriver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1) 
   * @param axis
   * @return
   */
  public double getCoDriverAxis(Axis axis) {
    return (coDriver.getRawAxis(axis.value) < -.1 || coDriver.getRawAxis(axis.value) > .1)
        ? coDriver.getRawAxis(axis.value)
        : 0;
  }

  /**
   * Accessor method to set codriver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setCoDriverRumble(double leftRumble, double rightRumble) {
    coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
    coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
   * accessor to get the true/false of the buttonNum 
   * on the coDriver control
   * @param buttonNum
   * @return the value of the button
   */
  public boolean getCoDriverButton(int buttonNum) {
    return coDriver.getRawButton(buttonNum);
  }

  public double getRobotForwardFull(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kLeftY)*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotForwardSlow(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kRightY)*0.5*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotLateralFull(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kLeftX)*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotLateralSlow(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kRightX)*0.5*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  /**
     * Gets the rotation value for the robot
     * Currently: from the driver's controller for swerve (LT and RT).
     * @param isVeloMode If velocity mode is being used.
     * @return The percent output if velocity mode is not being used, otherwise the velocity. 
     */
    public double getRobotRotation (boolean isVeloMode) {
      double raw = (this.getDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger));
      return -Math.copySign(Math.pow(raw, Constants.SwerveDriveConstants.DRIVER_ROT_SPEED_SCALE_EXPONENTIAL), raw) 
          * (isVeloMode ? Constants.SwerveDriveConstants.MAX_ROBOT_ROT_VELOCITY : Constants.SwerveDriveConstants.DRIVER_PERCENT_ROT_SPEED_SCALE_LINEAR);
  }

}