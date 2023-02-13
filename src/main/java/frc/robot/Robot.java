// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoElbow;
import frc.robot.commands.AutoShoulder;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ManualArmDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  Drivetrain drivetrain = new Drivetrain();
  Arm arm = new Arm();

  private int state = 0;
  private int m_rainbowFirstPixelHue = 0;
  private int rainbowtime = 0;
  private int fade = 0;
  private int direction = 0;
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(45);
  private int i = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureBindings();
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    switch(state){
      case 1:
          if(rainbowtime < 300){
            rainbow();
            rainbowtime++;
          } else{
            rainbowtime = 0;
            i = 0;
            state = 2;
          }
        break;
      case 2:
          if(i < 45){
            m_ledBuffer.setHSV(i, 0, 0, 0);
            i++;

          }else{
            state = 3;
            i = 0;
          }
        break;
      case 3:
        if(rainbowtime < 300){
          rainbowtime++;
          for (i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 80, 255, fade);
          }
          if(direction == 0){
            fade+=4;
            if(fade == 128){  
              direction = 1;
            }
          } else if(direction == 1) {
            fade-=4;
            if(fade == 0){
              direction = 0;
            }
          }
        } else{
          state = 4;
          i = 0;
          fade = 0;
          direction = 0;
          rainbowtime = 0;
        }
        break;
      case 4:
        if(i < 45){
          m_ledBuffer.setHSV(i, 0, 0, 0);
          i++;

        }else{
          state = 5;
          i = 0;
        }
        break;
      case 5:
        if((i < (m_ledBuffer.getLength()-1))){
          i++;
          m_ledBuffer.setHSV(i-1, 0, 0, 0);
          m_ledBuffer.setHSV(i, 80,255,128);
        } else{
          state = 6;
        }
        break;
      case 6:
        if((i > 1)){
          i--;
          m_ledBuffer.setHSV(i+1, 0, 0, 0);
          m_ledBuffer.setHSV(i, 80,255,128);
        }else{
          state = 7;
          i = 0;
        }
        break;
        case 7:
        if((i < (m_ledBuffer.getLength()-1))){
          i++;
          m_ledBuffer.setHSV(i-1, 0, 0, 0);
          m_ledBuffer.setHSV(i, 80,255,128);
        } else{
          state = 8;
        }
        break;
      case 8:
        if((i > 1)){
          i--;
          m_ledBuffer.setHSV(i+1, 0, 0, 0);
          m_ledBuffer.setHSV(i, 80,255,128);
        }else{
          state = 1;
          i = 0;
        }
        break;
      default:
        state = 1;
        
      break;
    }

    // Set the data
    m_led.setData(m_ledBuffer);
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
    arm.setDefaultCommand(new AutoElbow(arm));

    //Trigger elbowTest = new JoystickButton(operatorController, XboxController.Button.kA.value);
    //elbowTest.whileTrue(new AutoElbow(arm));

    //Trigger shoulderTest = new JoystickButton(operatorController, XboxController.Button.kB.value);
    //shoulderTest.whileTrue(new AutoShoulder(arm));


  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 1;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
