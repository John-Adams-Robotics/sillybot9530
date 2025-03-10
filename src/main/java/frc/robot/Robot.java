// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  //motor instances
  private final SparkMax m_frontleft;
  private final SparkMax m_backleft;
  private final SparkMax m_frontright;
  private final SparkMax m_backright;
  private final SparkMax m_shooter;

  //controller instances
  private final XboxController ctrl_driver;

  //kinematics instance
  private final DifferentialDriveKinematics obj_kinematics;

  //class-level constants
  private static final double cnst_trackwidth = Units.inchesToMeters(22.5); //width between centers of drivetrain wheels
  private static final double cnst_ctrldeadband = 0.25; //deadband for controller axis data

  //class-level variables for drive
  private double var_xaxis; //axis generated from transformed controller data
  private double var_zaxis; //axis generated from transformed controller data
  private double var_throttle; //axis generated from transformed controller data, used to limit X axis speed
  private boolean var_driver_a; //button state for shooter
  private boolean var_driver_b; //button state for shooter
  private boolean var_driver_left_shoulder; // button for a brake /* this is a break thingy*/
  public double startTime; // start time for the robot in auto
  public double time;// variable to keep track of time for autonomous
  public double timeAfter; // this is the time minus the start of autonmous
  public double distance_test; // REMOVE THIS LATER *********************
 
  


  //class-level variables for outtake
  private double var_shootspeed; //shooter speed variable

  // encoders
  public Encoder e_leftFront = new Encoder(0,1);
  public Robot() {

    //declare motor params, adjust node IDs as needed
    m_frontleft = new SparkMax(14, MotorType.kBrushed);
    m_backleft = new SparkMax(18, MotorType.kBrushed);
    m_frontright = new SparkMax(9, MotorType.kBrushed);
    m_backright = new SparkMax(2, MotorType.kBrushed);
    m_shooter = new SparkMax(10, MotorType.kBrushed);

    //controller params, adjust ports as needed
    ctrl_driver = new XboxController(0);

    //kinematic params
    obj_kinematics = new DifferentialDriveKinematics(cnst_trackwidth);
    
  }

  @Override
  public void robotPeriodic() {

    //telemetry
    SmartDashboard.putNumber("x axis", var_xaxis);
    SmartDashboard.putNumber("z axis", var_zaxis);
    SmartDashboard.putNumber("throttle", var_throttle);
    SmartDashboard.putBoolean("operator a button", var_driver_a);
    SmartDashboard.putBoolean("operator b button", var_driver_b);
    SmartDashboard.putNumber("startTime", startTime);
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("timeAfter", timeAfter);
    SmartDashboard.putBoolean("brrrrreak", var_driver_left_shoulder);
    SmartDashboard.putString("gey", ":3");
    SmartDashboard.putNumber("distance test", e_leftFront.getDistance());
    }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp(); // set time at auto start
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    double timeAfter = time - startTime;
    if (timeAfter < 8) {
      m_frontleft.set(-0.2);
      m_backleft.set(-0.2);
      m_frontright.set(0.2);
      m_backright.set(0.2);
    }
    else if (timeAfter < 10){
      m_frontleft.set(0);
      m_backleft.set(0);
      m_frontright.set(0);
      m_backright.set(0);
      m_shooter.set(0.4);
        }
   else {
          m_shooter.set(0);
        }
    }
  

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    DifferentialDriveWheelSpeeds var_wheelspeeds; //wheel speeds from kinematics

    //break check
    var_driver_left_shoulder = ctrl_driver.getLeftBumperButton();

    //axis transformations, consume and perform kinematics math using transformed axis data
    var_xaxis = -MathUtil.applyDeadband(ctrl_driver.getLeftY(), cnst_ctrldeadband) * MathUtil.clamp((var_throttle*var_throttle), 0.4, 1); //use throttle to limit speed for finer control
    var_zaxis = -MathUtil.applyDeadband(ctrl_driver.getLeftX(), cnst_ctrldeadband);
    var_throttle = 1 - MathUtil.applyDeadband(ctrl_driver.getRightTriggerAxis(), cnst_ctrldeadband); //subtract axis data from axis maximum to invert scale of axis
    var_wheelspeeds = obj_kinematics.toWheelSpeeds(new ChassisSpeeds(var_xaxis , 0, var_zaxis));

    //bools for shooter operations
    var_driver_a = ctrl_driver.getAButton();
    var_driver_b = ctrl_driver.getBButton();

    //shooter speed conditional, conditionals for each button must be nested, as mutiple separate else statements writing zero will not allow consistent movement
    if (var_driver_a || var_driver_b) {
      if (var_driver_a && var_driver_b == false) {var_shootspeed = 0.5;}
      if (var_driver_a == false && var_driver_b) {var_shootspeed = -0.5;}
    } else {
      var_shootspeed = 0;
    }

    //write speeds to motors for drive
    if (var_driver_left_shoulder == false) {
    m_frontleft.set(-var_wheelspeeds.leftMetersPerSecond);
    m_backleft.set(-var_wheelspeeds.leftMetersPerSecond);
    m_frontright.set(var_wheelspeeds.rightMetersPerSecond);
    m_backright.set(var_wheelspeeds.rightMetersPerSecond);
    }
    else if (var_driver_left_shoulder == true) {
      m_frontleft.set(0);
      m_backleft.set(0);
      m_frontright.set(0);
      m_backright.set(0);
    }
    
    //write speeds to shooter
    m_shooter.set(var_shootspeed);


  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
