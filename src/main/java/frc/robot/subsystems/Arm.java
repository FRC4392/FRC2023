// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  CANSparkMax Shoulder1 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax Shoulder2 = new CANSparkMax(22, MotorType.kBrushless);
  CANSparkMax Gripper = new CANSparkMax(35, MotorType.kBrushless);
  CANSparkMax Intake1 = new CANSparkMax(31, MotorType.kBrushless);
  CANSparkMax Intake2 = new CANSparkMax(32, MotorType.kBrushless);

  CANSparkMax Elbow = new CANSparkMax(23, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    Shoulder1.setSoftLimit(SoftLimitDirection.kForward, 116/8);
    Shoulder1.setSoftLimit(SoftLimitDirection.kReverse, -116/8);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kForward, true);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Shoulder2.follow(Shoulder1);

    Elbow.setSoftLimit(SoftLimitDirection.kForward, 116/3);
    Elbow.setSoftLimit(SoftLimitDirection.kReverse, -116/3);
    Elbow.enableSoftLimit(SoftLimitDirection.kForward, true);
    Elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Intake2.follow(Intake1, true);

    Intake1.setSmartCurrentLimit(15);
    Intake2.setSmartCurrentLimit(15);
    
    Gripper.setSmartCurrentLimit(10);
  }

  public void setShoulder(double Velocity){
    Shoulder1.set(Velocity*.3);
  }

  public void setElbow(double Velocity){
    Elbow.set(Velocity*.3);
  }

  public void setIntake(double velocity){
    Intake1.set(velocity);
  }

  public void setGripper(double velocity){
    Gripper.set(velocity);
    SmartDashboard.putNumber("gripperCurrent", Gripper.getOutputCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
