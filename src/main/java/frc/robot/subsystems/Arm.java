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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  /**
   *
   */
  private static final double dt = 0.02;
  private final RelativeEncoder shoulder1Encoder;
  private final RelativeEncoder elbowEncoder;
  private final CANCoder shoulCanCoder = new CANCoder(21);
  private final CANCoder elbowCanCoder = new CANCoder(22);
  private final SparkMaxPIDController shoulderPID;
  private final SparkMaxPIDController elbowPID;
  private final TrapezoidProfile.Constraints shoulderProfileConstraints = new Constraints(360, 200);
  private final TrapezoidProfile.Constraints elbowProfileContraints = new Constraints(360, 360*1.5);
  private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final CANSparkMax Shoulder1 = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax Shoulder2 = new CANSparkMax(22, MotorType.kBrushless);
  private final CANSparkMax Elbow = new CANSparkMax(23, MotorType.kBrushless);

  double ElbowPosition = 0.0;

  private TrapezoidProfile.State elbowSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulderSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State elbowGoal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State shoulderGoal = new TrapezoidProfile.State(0, 0);

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

    shoulder1Encoder.setPositionConversionFactor(360.0 / 98.38556505223172);
    elbowEncoder.setPositionConversionFactor(360.0 / 98.38556505223172);
    shoulder1Encoder.setVelocityConversionFactor((360.0 / 98.38556505223172) / 60);
    elbowEncoder.setVelocityConversionFactor((360.0 / 98.38556505223172) / 60);

    Shoulder1.setSoftLimit(SoftLimitDirection.kForward, 45);
    Shoulder1.setSoftLimit(SoftLimitDirection.kReverse, -45);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kForward, true);
    Shoulder1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    Elbow.setSoftLimit(SoftLimitDirection.kForward, 180);
    Elbow.setSoftLimit(SoftLimitDirection.kReverse, -180);
    Elbow.enableSoftLimit(SoftLimitDirection.kForward, true);
    Elbow.enableSoftLimit(SoftLimitDirection.kReverse, true);

    kP = 0.1;
    kI = 0.00;
    kD = 0.00;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

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

  public void setShoulder(double Velocity) {
    Shoulder1.setVoltage(((Velocity * 12) * .3) + getShoulderGravityFeedForward());
  }

  public void setElbow(double Velocity) {
    Elbow.setVoltage(((Velocity * 12) * .3) + getElbowGravityFeedForward());
  }

  public void setShoulderPosition(double position) {
    shoulderGoal = new TrapezoidProfile.State(position, 0);
  }

  public void setElbowPosition(double position) {
    elbowGoal = new TrapezoidProfile.State(position, 0);
  }

  public void resetElbowProfile() {
    elbowSetpoint = new TrapezoidProfile.State(elbowEncoder.getPosition(), 0);
  }

  public void resetShoulderProfile() {
    shoulderSetpoint = new TrapezoidProfile.State(shoulder1Encoder.getPosition(), 0);
  }

  public void setZero() {
    shoulder1Encoder.setPosition(shoulCanCoder.getAbsolutePosition());
    elbowEncoder.setPosition(elbowCanCoder.getAbsolutePosition() - shoulCanCoder.getAbsolutePosition());
  }

  public double getElbowPostition() {
    return elbowEncoder.getPosition();
  }

  public double getShoulderPostition() {
    return shoulder1Encoder.getPosition();
  }

  public boolean elbowInPostion(){
    return Math.abs(elbowGoal.position - elbowSetpoint.position) < 3;
  }
  public boolean shoulderInPostion(){
    return Math.abs(shoulderGoal.position - shoulderSetpoint.position) < 3;
  }

  public Command elbowPositionCommand(double position){
    return this.run(() -> setElbowPosition(position)).until(() -> elbowInPostion());
  }

  public Command shoulderPositionCommand(double position){
    return this.run(() -> setShoulderPosition(position)).until(() -> shoulderInPostion());
  }

  private void log(){
    SmartDashboard.putNumber("Shoulder Absolute position", shoulCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shoulder Inremental position", shoulder1Encoder.getPosition());
    SmartDashboard.putNumber("shoulder setpoint", shoulderSetpoint.position);
    SmartDashboard.putNumber("elbow setpoint", elbowSetpoint.position);
    SmartDashboard.putNumber("Elboe Absolute position", elbowCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elboe Inremental position", elbowEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    TrapezoidProfile profile = new TrapezoidProfile(elbowProfileContraints, elbowGoal, elbowSetpoint);
    elbowSetpoint = profile.calculate(dt);

    double elbowFeedForward = getElbowGravityFeedForward() + getElbowVelocityFeedForward(elbowSetpoint.velocity);

    elbowPID.setReference(elbowSetpoint.position, ControlType.kPosition, 0, elbowFeedForward);

    TrapezoidProfile shoulderProfile = new TrapezoidProfile(shoulderProfileConstraints, shoulderGoal, shoulderSetpoint);
    shoulderSetpoint = shoulderProfile.calculate(dt);

    double Shoulderfeedforward = getShoulderGravityFeedForward() + getShoulderVelocityFeedForward(shoulderSetpoint.velocity);

    shoulderPID.setReference(shoulderSetpoint.position, ControlType.kPosition, 0, Shoulderfeedforward);

    log();
  }

  private double getShoulderVelocityFeedForward(double velocity) {
    return (velocity * 0.0042) * 12.0;
  }

  private double getShoulderGravityFeedForward() {
    double Shoulderfeedforward = -.6 * Math.sin(Units.degreesToRadians(shoulder1Encoder.getPosition()));
    return Shoulderfeedforward;
  }

  private double getElbowVelocityFeedForward(double velocity) {
    return (velocity * 0.0042) * 12.0;
  }

  private double getElbowGravityFeedForward() {
    double Shoulderfeedforward = .5 * Math.sin(Units.degreesToRadians(shoulder1Encoder.getPosition()));
    return Shoulderfeedforward;
  }
}
