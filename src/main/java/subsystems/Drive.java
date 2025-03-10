// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    //motor instances
    private final SparkMax m_frontleft;
    private final SparkMax m_backleft;
    private final SparkMax m_frontright;
    private final SparkMax m_backright;
 
    //kinematics instance
    private final DifferentialDriveKinematics obj_kinematics;
  
    //class-level constants
    private static final double cnst_trackwidth = Units.inchesToMeters(22.5); //width between centers of drivetrain wheels
    private static final double cnst_ctrldeadband = 0.25; //deadband for controller axis data
  
    //class-level variables for drive
    private double var_xaxis; //axis generated from transformed controller data
    private double var_zaxis; //axis generated from transformed controller data
    private double var_throttle; //axis generated from transformed controller data, used to limit X axis speed
    private boolean var_driver_left_shoulder; // button for a brake /* this is a break thingy*/
    
    

    
  public Drive() {
    //kinematic params
    obj_kinematics = new DifferentialDriveKinematics(cnst_trackwidth);

    //declare motor params, adjust node IDs as needed
    m_frontleft = new SparkMax(14, MotorType.kBrushed);
    m_backleft = new SparkMax(18, MotorType.kBrushed);
    m_frontright = new SparkMax(9, MotorType.kBrushed);
    m_backright = new SparkMax(2, MotorType.kBrushed);
  }

  public void drive(double yAxis, double xAxis, double trigger, boolean bumper) {
        DifferentialDriveWheelSpeeds var_wheelspeeds; //wheel speeds from kinematics

    //break check
    var_driver_left_shoulder = bumper;

    //axis transformations, consume and perform kinematics math using transformed axis data
    var_xaxis = -MathUtil.applyDeadband(yAxis, cnst_ctrldeadband) * MathUtil.clamp((var_throttle*var_throttle), 0.4, 1); //use throttle to limit speed for finer control
    var_zaxis = -MathUtil.applyDeadband(xAxis, cnst_ctrldeadband);
    var_throttle = 1 - MathUtil.applyDeadband(trigger, cnst_ctrldeadband); //subtract axis data from axis maximum to invert scale of axis
    var_wheelspeeds = obj_kinematics.toWheelSpeeds(new ChassisSpeeds(var_xaxis , 0, var_zaxis));

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
    }

  public void manual_drive(double left, double right) {
      m_frontleft.set(-left);
      m_backleft.set(-left);
      m_frontright.set(right);
      m_backright.set(right);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
