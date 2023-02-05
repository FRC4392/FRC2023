// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArmDrive extends CommandBase {
  private final Arm robotArm;
  private final XboxController operatorController;
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
    robotArm.setElbow(operatorController.getLeftY());
    robotArm.setShoulder(operatorController.getRightY());
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