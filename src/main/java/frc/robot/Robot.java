// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import subsystems.Kele;
import subsystems.Drive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // shooter
  Kele Shooter = new Kele(); 

  // drive
  Drive sillydrive = new Drive();

  //controller instances
  private final XboxController ctrl_driver;

// variables for controller
private boolean var_driver_a; //button state for shooter
private boolean var_driver_b; //button state for shooter
private double var_driver_x_axis; // right stick x axis
private double var_driver_y_axis; // right stick y axis
private boolean var_driver_left_bumper; // left bumper
private double var_driver_right_trigger; //right trigger

// variables for time
public double startTime;
double time;
  public Robot() {
    //controller params, adjust ports as needed
    ctrl_driver = new XboxController(0);
    
  }

  @Override
  public void robotPeriodic() {

    //telemetry
    SmartDashboard.putBoolean("operator a button", var_driver_a);
    SmartDashboard.putBoolean("operator b button", var_driver_b);
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp());
    SmartDashboard.putString("gey", ":3");
    }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp(); // set time at auto start
  }

  @Override
  public void autonomousPeriodic() {
    time = Timer.getFPGATimestamp();
    double timeAfter = time - startTime; // FIX THIS LATER
    if (timeAfter < 8) {
      sillydrive.manual_drive(0.2, 0.2);
    }
    else if (timeAfter < 9) {
      sillydrive.manual_drive(0, 0);
    }
    else if (timeAfter < 11){
      Shooter.manual_shoot(0.5);
        }
   else {
        Shooter.manual_shoot(0);
        }
    //Test telem
    SmartDashboard.putNumber("TimeAfter", timeAfter);
    }
  

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    //bools for shooter operations
    var_driver_a = ctrl_driver.getAButton();
    var_driver_b = ctrl_driver.getBButton();

    //shooter n stuff
    var_driver_a = ctrl_driver.getAButton();
    var_driver_b = ctrl_driver.getBButton();
    Shooter.shoot(var_driver_a, var_driver_b);

    //driver data assignment
    var_driver_a = ctrl_driver.getAButton();
    var_driver_b = ctrl_driver.getBButton();
    var_driver_x_axis = ctrl_driver.getLeftX();
    var_driver_y_axis = ctrl_driver.getLeftY();
    var_driver_left_bumper = ctrl_driver.getLeftBumperButton();
    var_driver_right_trigger = ctrl_driver.getRightTriggerAxis();

    //driver n stuff
    sillydrive.drive(var_driver_y_axis, var_driver_x_axis, var_driver_right_trigger,
    var_driver_left_bumper);
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
