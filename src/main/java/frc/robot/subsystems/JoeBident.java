// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoeBident extends SubsystemBase {
  /** Creates a new JoeBident. */

  CANSparkMax CamellaHarris = new CANSparkMax(35, MotorType.kBrushless);
  //gripper
  CANSparkMax HunterBident = new CANSparkMax(31, MotorType.kBrushless);
  //intake1
  CANSparkMax JillBident = new CANSparkMax(32, MotorType.kBrushless);
  //intake2
  CANifier remoteIO = new CANifier(30);


  public JoeBident() {
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);

    HunterBident.setSmartCurrentLimit(25);
    HunterBident.setIdleMode(IdleMode.kBrake);
    JillBident.setSmartCurrentLimit(25);
    JillBident.setIdleMode(IdleMode.kBrake);
    
    CamellaHarris.setSmartCurrentLimit(10);
    CamellaHarris.setIdleMode(IdleMode.kBrake);
    //CamellaHarris.setSoftLimit(SoftLimitDirection.kReverse, 0);
    //CamellaHarris.enableSoftLimit(SoftLimitDirection.kReverse, true);
    CamellaHarris.setInverted(true);

  }
  public void setIntake(double velocity){
    HunterBident.set(velocity);
  }

  public void setGripper(double velocity){
    CamellaHarris.set(velocity);
    SmartDashboard.putNumber("gripperCurrent", CamellaHarris.getOutputCurrent());
  }

  public boolean getGripperProx(){
    return !remoteIO.getGeneralInput(GeneralPin.QUAD_A);
  }

  public boolean getGripperOccupied(){
    return remoteIO.getGeneralInput(GeneralPin.QUAD_B);
  }

  public boolean getIsGamePieceCube(){
    return false;//remoteIO.getGeneralInput(GeneralPin.QUAD_IDX);
  }

  public boolean getIsGamePieceCone(){
    return remoteIO.getGeneralInput(GeneralPin.LIMF);
  }

  public void setLEDColor(double R, double G, double B){
    remoteIO.setLEDOutput(G, LEDChannel.LEDChannelA);
    remoteIO.setLEDOutput(B, LEDChannel.LEDChannelC);
    remoteIO.setLEDOutput(R, LEDChannel.LEDChannelB);
  }

  public void stop(){
    setIntake(0);
    setGripper(0);
  }

  public Command autoGrabCommand(DoubleSupplier speed){
    return this.runEnd(
      () -> {
        if (getGripperProx()){
          setGripper(.5);
        } else {
          setGripper(0);
        }

        if(getGripperOccupied()){
          setIntake(speed.getAsDouble() * .25);
        } else {
          setIntake(speed.getAsDouble());
        }
      }, 
      () -> {
        if (getGripperOccupied() || getGripperProx()){
          setIntake(.1);
          setGripper(.1);
        } else {
          setIntake(0);
          setGripper(0);
        }
      });
  }

  public Command ejectWhileOpeningCommand(DoubleSupplier speed){
    return this.runEnd(() -> {
      setGripper(-.1);
      setIntake(speed.getAsDouble());
    }, () -> stop());
  }

  public Command ejectWithoutOpeningCommand(DoubleSupplier speed){
    return this.runEnd(() -> {
      setIntake(speed.getAsDouble());
    }, () -> stop());
  }

  public Command openCommand(){
    return this.runEnd(() -> {
      setGripper(-.5);
    }, () -> stop());
  }

  public Command closeCommand(){
    return this.runEnd(() -> {
      setGripper(.5);
    }, () -> stop());
  }

  public void setCone(){
    setLEDColor(.5, .5, 0);
  }

  public void setCube(){
    setLEDColor(.5, 0, .5);
  }

  public void setNeutral(){
    if (DriverStation.getAlliance() == Alliance.Blue){
    setLEDColor(0, 0, 1);
    } else {
      setLEDColor(1, 0, 0);
    }
  }

  public Command cubeCommand(){
    return this.run(() -> setCube());
  }

  public Command coneCommand(){
    return this.run(() -> setCone());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
