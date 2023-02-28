// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
  CANifier remoteIO = new CANifier(30);


  public JoeBident() {
    JillBident.follow(HunterBident, true);

    HunterBident.setSmartCurrentLimit(25);
    HunterBident.setIdleMode(IdleMode.kBrake);
    JillBident.setSmartCurrentLimit(25);
    JillBident.setIdleMode(IdleMode.kBrake);
    
    CamellaHarris.setSmartCurrentLimit(15);
    CamellaHarris.setIdleMode(IdleMode.kBrake);

    remoteIO.setLEDOutput(255, LEDChannel.LEDChannelB);

    remoteIO.getGeneralInput(GeneralPin.QUAD_A);
  }
  public void setIntake(double velocity){
    HunterBident.set(velocity);
  }

  public void setGripper(double velocity){
    CamellaHarris.set(velocity);
    SmartDashboard.putNumber("gripperCurrent", CamellaHarris.getOutputCurrent());
  }

  public boolean getGripperProx(){
    return remoteIO.getGeneralInput(GeneralPin.QUAD_A);
  }

  public boolean getGripperOccupied(){
    return remoteIO.getGeneralInput(GeneralPin.QUAD_B);
  }

  public boolean getIsGamePieceCube(){
    return remoteIO.getGeneralInput(GeneralPin.QUAD_IDX);
  }

  public boolean getIsGamePieceCone(){
    return remoteIO.getGeneralInput(GeneralPin.LIMF);
  }

  public void setLEDColor(double R, double G, double B){
    remoteIO.setLEDOutput(G, LEDChannel.LEDChannelA);
    remoteIO.setLEDOutput(B, LEDChannel.LEDChannelB);
    remoteIO.setLEDOutput(R, LEDChannel.LEDChannelC);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getIsGamePieceCone()){
      setLEDColor(1, 1, 0);
    } else if (getIsGamePieceCube()){
      setLEDColor(.5, 0, .5);
    } else {
      setLEDColor(0, 0, 1);
    }
  }
}
