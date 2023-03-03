// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.JoeBident;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static Command getSimpleAutoCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),  bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)),new FollowPathPlannerPath(PathPlanner.loadPath("QuickStart", 3, 1),true, drivetrain));
    //  
  }
}
