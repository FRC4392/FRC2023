// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.deceivers.util.JoystickHelper;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArmDrive extends CommandBase {
  private final Arm robotArm;
  private final XboxController operatorController;

  private JoystickHelper xHelper = new JoystickHelper(0);
  private JoystickHelper yHelper = new JoystickHelper(0);
  private JoystickHelper rotHelper = new JoystickHelper(0);
  private JoystickHelper xrHelper = new JoystickHelper(0);
  private JoystickHelper yrHelper = new JoystickHelper(0);
  /** Creates a new ManualArmDrive. */
  public ManualArmDrive(Arm arm, XboxController controller) {
    robotArm = arm;
    operatorController = controller;
    

    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    robotArm.setElbow(xHelper.setInput(operatorController.getLeftY()).applyDeadband(.1).value);
    robotArm.setShoulder(yHelper.setInput(operatorController.getRightY()).applyDeadband(.1).value);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotArm.setElbow(0);
    robotArm.setShoulder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
