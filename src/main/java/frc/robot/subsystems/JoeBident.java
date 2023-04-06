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
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    CamellaHarris.restoreFactoryDefaults();
    HunterBident.restoreFactoryDefaults();
    JillBident.restoreFactoryDefaults();

    
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);

    HunterBident.setSmartCurrentLimit(25);
    HunterBident.setIdleMode(IdleMode.kBrake);
    JillBident.setSmartCurrentLimit(25);
    JillBident.setIdleMode(IdleMode.kBrake);
    
    CamellaHarris.setSmartCurrentLimit(10);
    CamellaHarris.setIdleMode(IdleMode.kBrake);
    CamellaHarris.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    //CamellaHarris.setSoftLimit(SoftLimitDirection.kReverse, 0);
    CamellaHarris.enableSoftLimit(SoftLimitDirection.kReverse, false);
    CamellaHarris.setInverted(true);

    HunterBident.burnFlash();
    JillBident.burnFlash();
    CamellaHarris.burnFlash();

  }
  public void setIntake(double velocity){
    HunterBident.set(velocity);
  }

  public void setGripper(double velocity){
    CamellaHarris.set(velocity);
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
int flashy = 0;
int no = 0;
  public Command autoGrabCommand(DoubleSupplier speed){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);

    return this.runEnd(
      () -> {
        
        if(no == 1){
          flashy++;
         setNeutral();
        } else {
          flashy--;
          setLEDColor(0, 0, 0);
        }
        if(flashy > 10){
          no = 0;
        } else if(flashy<0){
          no = 1;
        }
        if (getGripperProx()){
          setGripper(.8);
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
        setLEDColor(0, 0, 0);
        if (getGripperOccupied()){
          setIntake(.3);
          setGripper(.2);
        } else {
          setIntake(0);
          setGripper(0);
        }
      });
  }

  public Command autoIntakeCommand(Double speed){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);
    return this.runEnd(
      () -> {
        setLEDColor(0, 255, 0);
        if (getGripperProx()){
          setGripper(.8);
        } else {
          setGripper(0);
        }

        if(getGripperOccupied()){
          setIntake(speed *.25);
        } else {
          setIntake(speed);
        }
      }, 
      () -> {
        setLEDColor(0, 0, 0);
        if (getGripperOccupied() || getGripperProx()){
          setIntake(.2);
          setGripper(.2);
        } else {
          setIntake(0);
          setGripper(0);
        }
      });
  }

  public Command grabCubeCommand(Double speed){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);
    return this.runEnd(() -> {
      setLEDColor(0, 255, 0);
      if (getGripperProx()){
        setGripper(-.2);
      } else {
        setGripper(-.2);
      }

      if(getGripperOccupied()){
        setIntake(speed *.25);
      } else {
        setIntake(speed);
      }
    }, 
    () -> {
      setLEDColor(0, 0, 0);
      if (getGripperOccupied() || getGripperProx()){
        setIntake(.25);
        setGripper(.2);
      } else {
        setIntake(0);
        setGripper(0);
      }
    });
  }
  public Command ejectWhileOpeningCommand(DoubleSupplier speed){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);
    return this.runEnd(() -> {
      setGripper(-.1);
      setIntake(speed.getAsDouble());
    }, () -> stop());
  }

  public Command ejectWithoutOpeningCommand(DoubleSupplier speed){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);
    return this.runEnd(() -> {
      setIntake(speed.getAsDouble());
    }, () -> stop());
  }

  public Command openCommand(){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);
    return this.runEnd(() -> {
      setGripper(-.5);
    }, () -> stop());
  }

  public Command closeCommand(){
    HunterBident.setInverted(true);
    JillBident.follow(HunterBident, true);
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
    if (DriverStation.getAlliance() == Alliance.Red){
    setLEDColor(1, 0, 0);
    } else {
      setLEDColor(0, 0, 1);
    }
  }

  public Command cubeCommand(){
    return this.run(() -> setCube());
  }

  public Command coneCommand(){
    return this.run(() -> setCone());
  }

  public Command actuallyNeutral(){
    return this.run(() -> setNeutral());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
