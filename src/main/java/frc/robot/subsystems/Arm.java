// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final RelativeEncoder shoulder1Encoder;
  private final RelativeEncoder elbowEncoder;
  private final CANCoder shoulCanCoder = new CANCoder(21);
  private final CANCoder elbowCanCoder = new CANCoder(22);
  private final SparkMaxPIDController shoulderPID;
  private final SparkMaxPIDController elbowPID;

  CANSparkMax Shoulder1 = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax Shoulder2 = new CANSparkMax(22, MotorType.kBrushless);

  CANSparkMax Elbow = new CANSparkMax(23, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {

    Shoulder1.restoreFactoryDefaults();
    Shoulder2.restoreFactoryDefaults();
    Elbow.restoreFactoryDefaults();

    shoulder1Encoder = Shoulder1.getEncoder();
    elbowEncoder = Elbow.getEncoder();

    shoulderPID = Shoulder1.getPIDController();
    elbowPID = Elbow.getPIDController();

    Elbow.setInverted(true);
    Shoulder1.setInverted(true);
    Shoulder2.setInverted(true);

    shoulder1Encoder.setPositionConversionFactor(360.0/98.38556505223172);
    elbowEncoder.setPositionConversionFactor(360.0/98.38556505223172);

    Shoulder1.setSoftLimit(SoftLimitDirection.kForward, 45);
    Shoulder1.setSoftLimit(SoftLimitDirection.kReverse, -45);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kForward, true);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Elbow.setSoftLimit(SoftLimitDirection.kForward, 120);
    Elbow.setSoftLimit(SoftLimitDirection.kReverse, -120);
    Elbow.enableSoftLimit(SoftLimitDirection.kForward, true);
    Elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Shoulder2.follow(Shoulder1);
    setZero();
    
  }

  public void setShoulder(double Velocity){
    Shoulder1.set(Velocity*.3);
  }

  public void setElbow(double Velocity){
    Elbow.set(Velocity*.3);
  }

 public void setZero(){
  shoulder1Encoder.setPosition(shoulCanCoder.getAbsolutePosition());
  elbowEncoder.setPosition(elbowCanCoder.getAbsolutePosition()+shoulCanCoder.getAbsolutePosition());
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
