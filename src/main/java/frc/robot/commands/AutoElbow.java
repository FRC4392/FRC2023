// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AutoElbow extends CommandBase {
  Arm arm;
  /** Creates a new AutoElbow. */
  public AutoElbow(Arm armInstance) {
    arm = armInstance;
    addRequirements(armInstance);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetElbow();
    arm.resetShoulder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setElbowPosition(0);
    arm.setShoulderPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setElbow(0);
    arm.setShoulder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
