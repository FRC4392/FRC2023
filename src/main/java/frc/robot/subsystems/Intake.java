// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import org.opencv.objdetect.FaceDetectorYN;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {


  private static final double dt = .02;
  private final RelativeEncoder pivotEncoder;
  private final SparkMaxPIDController pivotPID;
  private final TrapezoidProfile.Constraints pivotProfileConstraints = new Constraints(360, 360);
  private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final CANSparkMax pivot = new CANSparkMax(44, MotorType.kBrushless);
  private final CANSparkMax roller = new CANSparkMax(43, MotorType.kBrushless);
  private final CANSparkMax left = new CANSparkMax(41, MotorType.kBrushless);
  private final CANSparkMax right = new CANSparkMax(42, MotorType.kBrushless);

  private TrapezoidProfile.State pivotSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State(0, 0);


  /** Creates a new Intake. */
  public Intake() {

    pivot.restoreFactoryDefaults();
    roller.restoreFactoryDefaults();
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    roller.setInverted(true);
    right.setInverted(true);

    pivotEncoder = pivot.getEncoder();

    pivotPID = pivot.getPIDController();

    pivotEncoder.setPositionConversionFactor(360.0/75.23);

    pivotPID.setOutputRange(.5, .5);

    kP = 0.1;
    kI = 0.00;
    kD = 0.00;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    pivotPID.setP(kP);
    pivotPID.setI(kI);
    pivotPID.setD(kD);
    pivotPID.setIZone(kIz);
    pivotPID.setFF(kFF);
    pivotPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setIntake(double speed){
    roller.set(speed);
    left.set(speed);
    right.set(speed);

  }

  public void setAngle(double angle){
    pivotGoal = new TrapezoidProfile.State(angle, 0);
  }

  public Command getIntakeCommand(){
    return this.runEnd(() -> setIntake(.25), () -> setIntake(0));
  }

  public Command getIntakePivotCommand(double angle){
    return this.run(() -> setAngle(angle)).until(()-> pivotInPosition());
  }

  private boolean pivotInPosition() {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    TrapezoidProfile profile = new TrapezoidProfile(pivotProfileConstraints, pivotGoal, pivotSetpoint);
    pivotSetpoint = profile.calculate(dt);

    double elbowFeedForward = getGravityFeedForward() + getVelocityFeedForward(pivotSetpoint.velocity);

    pivotPID.setReference(pivotSetpoint.position, ControlType.kPosition, 0, elbowFeedForward);
  }

  private int getVelocityFeedForward(double velocity) {
    return 0;
  }

  private int getGravityFeedForward() {
    return 0;
  }
}
