// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.JoeBident;

public class manualGripper2 extends CommandBase {
  JoeBident gripper;
  DoubleSupplier intake;
  /** Creates a new manualGripper. */
  public manualGripper2(JoeBident bident, DoubleSupplier intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    gripper = bident;
    intake = intakeSpeed;
    addRequirements(bident);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.setIntake(intake.getAsDouble());
    gripper.setGripper(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
