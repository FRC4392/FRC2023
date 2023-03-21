// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.JoeBident;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static Command getLoadingStationCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", intake.getIntakePivotCommand(95.0).andThen(arm.elbowPositionCommand(13.5).andThen(arm.shoulderPositionCommand(20).alongWith(bident.autoIntakeCommand(.75).alongWith(intake.getIntakeCommand())))));
    eventMap.put("ScorePosition", arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-102)).andThen(intake.getIntakePivotCommand(0.0)).andThen(arm.shoulderPositionCommand(-17)));
    eventMap.put("IntakeAgain", arm.shoulderPositionCommand(0).alongWith(intake.getIntakePivotCommand(95)).andThen(arm.elbowPositionCommand(13.5).andThen(arm.shoulderPositionCommand(20).alongWith(bident.autoIntakeCommand(0.75).alongWith(intake.getIntakeCommand())))));
    eventMap.put("Retract", arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(0)).andThen(intake.getIntakePivotCommand(0.0)));

    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup("LoadingStation", 5, 2.25);


    FollowPathWithEvents firstPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(0), DriverStation.getAlliance()), true, drivetrain), paths.get(0).getMarkers(), eventMap);

      FollowPathWithEvents secondPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(1), DriverStation.getAlliance()), true, drivetrain), paths.get(1).getMarkers(), eventMap);

    return Commands.sequence(arm.resetEncoder(), bident.autoGrabCommand(()->0.1).withTimeout(0.05).raceWith(intake.getIntakePivotCommand(95)), arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(0.1), bident.openCommand().raceWith(arm.shoulderPositionCommand(0.0)),
        firstPath,
        bident.ejectWhileOpeningCommand(() -> -.4).withTimeout(.1),
        secondPath);
  }

  public static Command getBumpPart1Command(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", intake.getIntakePivotCommand(95.0).andThen(arm.elbowPositionCommand(13.5).andThen(arm.shoulderPositionCommand(20).alongWith(bident.autoIntakeCommand(.75).alongWith(intake.getIntakeCommand())))));
    eventMap.put("ScorePosition", arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-102)).andThen(intake.getIntakePivotCommand(0.0)).andThen(arm.shoulderPositionCommand(-17)));
    eventMap.put("IntakeAgain", arm.shoulderPositionCommand(0).alongWith(intake.getIntakePivotCommand(95)).andThen(arm.elbowPositionCommand(13.5).andThen(arm.shoulderPositionCommand(20).alongWith(bident.autoIntakeCommand(0.75).alongWith(intake.getIntakeCommand())))));
    eventMap.put("Retract", arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(0)).andThen(intake.getIntakePivotCommand(0.0)));

    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup("BumpPart1", 5, 2.25);


    FollowPathWithEvents firstPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(0), DriverStation.getAlliance()), true, drivetrain), paths.get(0).getMarkers(), eventMap);

      FollowPathWithEvents secondPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(1), DriverStation.getAlliance()), true, drivetrain), paths.get(1).getMarkers(), eventMap);

    return Commands.sequence(arm.resetEncoder(), bident.autoGrabCommand(()->0.1).withTimeout(0.05).raceWith(intake.getIntakePivotCommand(95)), arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(0.1), bident.openCommand().raceWith(arm.shoulderPositionCommand(0.0)),
        firstPath,
        bident.ejectWhileOpeningCommand(() -> -.4).withTimeout(.1),
        secondPath);
  }

  public static Command getBalanceCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)),
        new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath("Balance", 3, 1), DriverStation.getAlliance()), true, drivetrain));
  }

  public static Command getBumpCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)),
        new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath("Bump", 3, 1), DriverStation.getAlliance()), true, drivetrain));
  }

  public static Command getStupidCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)));
  }
}
