// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.manualGripper;
import frc.robot.commands.manualGripper2;
import frc.robot.commands.manualGripper3;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.JoeBident;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  XboxController driverController = new XboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  Drivetrain drivetrain = new Drivetrain();
  Arm arm = new Arm();
  JoeBident bident = new JoeBident();

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureBindings();

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
    Trigger CubeTrigger = operatorController.povLeft();
    Trigger highGoalConeTrigger = operatorController.y().and(CubeTrigger.negate());
    Trigger highGoalCubeTrigger = operatorController.y().and(CubeTrigger);
    Trigger MidGoalConeTrigger = operatorController.x().and(CubeTrigger.negate());
    Trigger MidGoalCubeTrigger = operatorController.x().and(CubeTrigger);

    Trigger shelfTrigger = operatorController.a().and(CubeTrigger.negate());
    Trigger shelfTriggerBackwards = operatorController.a().and(CubeTrigger);

    operatorController.b().onTrue(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(0)));
    highGoalConeTrigger.onTrue(arm.elbowPositionCommand(130.0).andThen(arm.shoulderPositionCommand(37.0)));
    highGoalCubeTrigger.onTrue(arm.elbowPositionCommand(90).andThen(arm.shoulderPositionCommand(17)));
    MidGoalConeTrigger.onTrue(arm.elbowPositionCommand(84).andThen(arm.shoulderPositionCommand(14)));
    MidGoalCubeTrigger.onTrue(arm.elbowPositionCommand(66).andThen(arm.shoulderPositionCommand(0)));


    shelfTrigger.onTrue(arm.shoulderPositionCommand(12.0).andThen(arm.elbowPositionCommand(-95)));
    shelfTriggerBackwards.onTrue(arm.shoulderPositionCommand(-12).andThen(arm.elbowPositionCommand(95)));

    DoubleSupplier intakeSpeed = () -> operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis();


    bident.setDefaultCommand(new manualGripper3(bident, intakeSpeed));
    operatorController.leftBumper().whileTrue(new manualGripper(bident, intakeSpeed));
    operatorController.rightBumper().whileTrue(new manualGripper2(bident, intakeSpeed));
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
  }
}
