// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  CANSparkMax Shoulder1 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax Shoulder2 = new CANSparkMax(22, MotorType.kBrushless);

  CANSparkMax Elbow = new CANSparkMax(23, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    Shoulder1.setSoftLimit(SoftLimitDirection.kForward, 116/8);
    Shoulder1.setSoftLimit(SoftLimitDirection.kReverse, -116/8);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kForward, true);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Shoulder2.follow(Shoulder1);

    Elbow.setSoftLimit(SoftLimitDirection.kForward, 116/4);
    Elbow.setSoftLimit(SoftLimitDirection.kReverse, -116/4);
    Elbow.enableSoftLimit(SoftLimitDirection.kForward, true);
    Elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void setShoulder(double Velocity){
    Shoulder1.set(Velocity*.3);
  }

  public void setElbow(double Velocity){
    Elbow.set(Velocity*.3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
