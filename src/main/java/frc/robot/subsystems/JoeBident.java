// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoeBident extends SubsystemBase {
  /** Creates a new JoeBident. */

  CANSparkMax CamellaHarris = new CANSparkMax(35, MotorType.kBrushless);
  //gripper
  CANSparkMax HunterBident = new CANSparkMax(31, MotorType.kBrushless);
  //intake1
  CANSparkMax JillBident = new CANSparkMax(32, MotorType.kBrushless);
  //intake2


  public JoeBident() {
    JillBident.follow(HunterBident, true);

    HunterBident.setSmartCurrentLimit(25);
    JillBident.setSmartCurrentLimit(25);
    
    CamellaHarris.setSmartCurrentLimit(15);
  }
  public void setIntake(double velocity){
    HunterBident.set(velocity);
  }

  public void setGripper(double velocity){
    CamellaHarris.set(velocity);
    SmartDashboard.putNumber("gripperCurrent", CamellaHarris.getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
