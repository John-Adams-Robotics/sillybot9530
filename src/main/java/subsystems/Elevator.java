// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // motor declaration
  SparkMax m_elevator = new SparkMax(5, MotorType.kBrushed);
  
  // variables
  public boolean button_a;
  public boolean button_b;
  public final double speed = 0.5;
  public final double max = 0;
  public final double min = 0;
  public final double rebound = 0.4;
  

  // encoder
  Encoder e_elevator = new Encoder(0,1);
  public Elevator() {

  }
  public void move(boolean a, boolean b) {
    
    button_a = a;
    button_b = b;
    if (e_elevator.getDistance() <= max && e_elevator.getDistance() >= min) {
      if (button_a == true && button_b == false) {
        m_elevator.set(speed);
      }
      else if (button_a == false && button_b == true) {
        m_elevator.set(-speed);
      }
      else {
        m_elevator.set(0);
      }
    }
    else if (e_elevator.getDistance() > max) {
      m_elevator.set(-rebound);
    }
    else if (e_elevator.getDistance() < min) {
      m_elevator.set(rebound);
    }
    else {
      m_elevator.set(0);
      System.out.println("WHAT THE FRICK DID YOU DOOOOOOOOOO");
      System.out.println(e_elevator.getDistance());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
