// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Kele extends SubsystemBase {
  // motor delaration
  SparkMax m_shooter = new SparkMax(10, MotorType.kBrushed);

  //variables
  private boolean button_a;
  private boolean button_b;
  public  double var_shootspeed;

  // THIS IS THE SPEED THAT THE SHOOTER GOES AT
  final public double speed = 0.5;

  public Kele() {}

  //shoter function
  public void shoot(boolean a, boolean b) {
    button_a = a;
    button_b = b;

    //shooter speed conditional, conditionals for each button must be nested, as mutiple separate else statements writing zero will not allow consistent movement
    if (button_a || button_b) {
      if (button_a && button_b == false) {var_shootspeed = speed;}
      if (button_a == false && button_b) {var_shootspeed = -speed;}
    } else {
      var_shootspeed = 0;
    }

     //write speeds to shooter
     m_shooter.set(var_shootspeed);
  }
  @Override
  public void periodic() {
    
  }
}
