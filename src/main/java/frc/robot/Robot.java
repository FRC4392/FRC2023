// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.resetArmEncoders;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.JoeBident;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;

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
  Intake intake = new Intake();
  Limelight limelight = new Limelight();
  LED led = new LED();

  SendableChooser<String> chooser = new SendableChooser<>();

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Stupid", "Stupid");
    chooser.addOption("LoadingStation", "LoadingStation");
    chooser.addOption("Balance", "Balance");
    chooser.addOption("Bump", "Bump");

    SmartDashboard.putData(chooser);

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
  public void disabledInit() {
    led.setMode(0);
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.getAlliance() == Alliance.Blue){
    bident.setLEDColor(0, 0, 1);
    } else {
      bident.setLEDColor(1, 0, 0);
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    switch(chooser.getSelected()){
      case "LoadingStation":
        m_autonomousCommand = Autos.getLoadingStationCommand(arm, bident, drivetrain, intake);
      break;
      case "Balance":
        m_autonomousCommand = Autos.getBalanceCommand(arm, bident, drivetrain, intake);
      break;
      case "Bump":
        m_autonomousCommand = Autos.getBumpCommand(arm, bident, drivetrain, intake);
      break;
      case "Stupid":
        m_autonomousCommand = Autos.getStupidCommand(arm, bident, drivetrain, intake);
      break;
      default:
        m_autonomousCommand = null;
    }
    // schedule the autonomous command (example)
    led.setMode(1);
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
    led.setMode(2);
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
    led.setMode(3);
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    led.setMode(4);
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
    Trigger shelfTrigger = operatorController.b().and(CubeTrigger.negate());
    Trigger lowGoalConeTrigger = operatorController.a().and(CubeTrigger.negate());
    Trigger lowGoalCubeTrigger = operatorController.a().and(CubeTrigger);

    Trigger groundIntake = operatorController.leftStick().and(CubeTrigger.negate());;
    Trigger groundOuttake = operatorController.leftStick().and(CubeTrigger);

    Trigger cubeIndicatorTrigger = operatorController.povDown();
    Trigger coneIndicatorTrigger = operatorController.povUp();

    BooleanSupplier brakeSupplier = () -> driverController.getXButton();
    BooleanSupplier gripperSupplier = () -> bident.getGripperOccupied();

    Trigger brake = new Trigger(brakeSupplier);
    Trigger gripperReady = new Trigger(gripperSupplier);

    brake.whileTrue(drivetrain.brakeCommand());
    gripperReady.whileTrue(led.accyGreem());

    operatorController.rightStick().onTrue(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(0)).andThen(intake.getIntakePivotCommand(0.0)));
    highGoalConeTrigger.onTrue(arm.elbowPositionCommand(-132.0).andThen(arm.shoulderPositionCommand(-37.0)));
    highGoalCubeTrigger.onTrue(arm.elbowPositionCommand(-102).andThen(arm.shoulderPositionCommand(-17)));
    MidGoalConeTrigger.onTrue(arm.elbowPositionCommand(-102).andThen(arm.shoulderPositionCommand(-8)));
    MidGoalCubeTrigger.onTrue(arm.elbowPositionCommand(-78).andThen(arm.shoulderPositionCommand(0)));
    lowGoalConeTrigger.onTrue(arm.elbowPositionCommand(-12).andThen(arm.shoulderPositionCommand(-20.0)));
    lowGoalCubeTrigger.onTrue(arm.elbowPositionCommand(-12).andThen(arm.shoulderPositionCommand(-20.0)));

    shelfTrigger.onTrue(arm.elbowPositionCommand(-98).andThen(arm.shoulderPositionCommand(12.0)));

     cubeIndicatorTrigger.whileTrue(bident.cubeCommand());
     coneIndicatorTrigger.whileTrue(bident.coneCommand());

     operatorController.back().whileTrue(arm.resetEncoder());

    DoubleSupplier intakeSpeed = () -> operatorController.getLeftTriggerAxis() - operatorController.getRightTriggerAxis();
      
    // bident.setDefaultCommand(new manualGripper3(bident, intakeSpeed));

    operatorController.leftTrigger(0).whileTrue(bident.autoGrabCommand(intakeSpeed).alongWith(intake.getIntakeCommand()));
    operatorController.rightTrigger(0).whileTrue(bident.ejectWithoutOpeningCommand(intakeSpeed));

    groundIntake.whileTrue(intake.getIntakePivotCommand(95.0).andThen(arm.elbowPositionCommand(15).andThen(arm.shoulderPositionCommand(17).alongWith(bident.autoIntakeCommand(0.5).alongWith(intake.getIntakeCommand())))));
    groundIntake.onFalse(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(0)).andThen(intake.getIntakePivotCommand(0.0)));
    groundOuttake.whileTrue(intake.getIntakePivotCommand(95.0).andThen(arm.elbowPositionCommand(18).andThen(arm.shoulderPositionCommand(15).alongWith(intake.getOuttakeCommand()))));

    operatorController.leftBumper().whileTrue(bident.closeCommand());
    operatorController.rightBumper().whileTrue(bident.openCommand());
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));
  }
}
