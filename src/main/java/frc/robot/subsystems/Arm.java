// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final RelativeEncoder shoulder1Encoder;
  private final RelativeEncoder elbowEncoder;
  private final CANCoder shoulCanCoder = new CANCoder(21);
  private final CANCoder elbowCanCoder = new CANCoder(22);
  private final SparkMaxPIDController shoulderPID;
  private final SparkMaxPIDController elbowPID;
  private final ProfiledPIDController elbowPIDController;
  private final ProfiledPIDController shoulderPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

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
    shoulder1Encoder.setVelocityConversionFactor((360.0/98.38556505223172)/60);
    elbowEncoder.setVelocityConversionFactor((360.0/98.38556505223172)/60);

    Shoulder1.setSoftLimit(SoftLimitDirection.kForward, 45);
    Shoulder1.setSoftLimit(SoftLimitDirection.kReverse, -45);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kForward, true);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Elbow.setSoftLimit(SoftLimitDirection.kForward, 120);
    Elbow.setSoftLimit(SoftLimitDirection.kReverse, -120);
    Elbow.enableSoftLimit(SoftLimitDirection.kForward, true);
    Elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
    kP = 0.00; 
    kI = 0.00;
    kD = 0; 
    kIz = 0; 
    kFF = .0042;
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxVel = 360/1; // rpm
    maxAcc = 360.0;
    
    elbowPIDController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(maxVel, maxAcc));
    shoulderPIDController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(maxVel, maxVel));

    elbowPID.setP(kP);
    elbowPID.setI(kI);
    elbowPID.setD(kD);
    elbowPID.setIZone(kIz);
    elbowPID.setFF(kFF);
    elbowPID.setOutputRange(kMinOutput, kMaxOutput);

    shoulderPID.setP(kP);
    shoulderPID.setI(kI);
    shoulderPID.setD(kD);
    shoulderPID.setIZone(kIz);
    shoulderPID.setFF(kFF);
    shoulderPID.setOutputRange(kMinOutput, kMaxOutput);

    Shoulder2.follow(Shoulder1);
    setZero();
  }

  public void setShoulder(double Velocity){
    double kFFArb = .5*Math.sin(Units.degreesToRadians(shoulCanCoder.getAbsolutePosition()));
    Shoulder1.setVoltage(((Velocity*12)*.3)+kFFArb);
  }

  public void setElbow(double Velocity){
    double kFFArb = .5*Math.sin(Units.degreesToRadians(elbowCanCoder.getAbsolutePosition()-shoulCanCoder.getAbsolutePosition()));
    Elbow.setVoltage(((Velocity*12)*.3)+kFFArb);
  }

  public void setShoulderPosition(double position){
    double setpoint = shoulderPIDController.calculate(shoulCanCoder.getAbsolutePosition(), position);
    shoulderPID.setReference(setpoint, ControlType.kVelocity, 0, .5*Math.sin(Units.degreesToRadians(shoulCanCoder.getAbsolutePosition())));
  }
  public void setElbowPosition(double position){
    double setpoint = elbowPIDController.calculate(elbowCanCoder.getAbsolutePosition()-shoulCanCoder.getAbsolutePosition(), position);
    elbowPID.setReference(setpoint, ControlType.kVelocity, 0, .5*Math.sin(Units.degreesToRadians(elbowCanCoder.getAbsolutePosition())));
  }

  public void resetElbow(){
    elbowPIDController.reset(elbowCanCoder.getAbsolutePosition());
  }

  public void resetShoulder(){
    shoulderPIDController.reset(shoulCanCoder.getAbsolutePosition());
  }

 public void setZero(){
  shoulder1Encoder.setPosition(shoulCanCoder.getAbsolutePosition());
  elbowEncoder.setPosition(elbowCanCoder.getAbsolutePosition()-shoulCanCoder.getAbsolutePosition());
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
