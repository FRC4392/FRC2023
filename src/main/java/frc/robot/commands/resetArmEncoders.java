// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED;

public class resetArmEncoders extends CommandBase {

  private final Arm arm;
  private final LED leds;

  /** Creates a new resetArmEncoders. */
  public resetArmEncoders(Arm newArm, LED newLEDS) {
    arm = newArm;
    leds = newLEDS;

    addRequirements(arm, leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setMode(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setZero();

    if (arm.isZero()) {
      leds.setLEDGreen();
    } else {
      leds.setLEDBlack();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setMode(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
