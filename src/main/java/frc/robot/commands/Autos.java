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
    eventMap.put("IntakePosition", intake.getIntakePivotCommand(98.0).andThen(arm.elbowPositionCommand(20.0)).andThen(arm.shoulderPositionCommand(20).andThen(arm.elbowPositionCommand(11.75))));
    eventMap.put("Intake", bident.grabCubeCommand(1.0).alongWith(intake.getIntakeCommand()));
    eventMap.put("ScorePosition", arm.elbowPositionCommand(20).andThen(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-102).until((() -> arm.getElbowPostition() < 8))).andThen(intake.getIntakePivotCommand(0.0)).andThen(arm.shoulderPositionCommand(-17))));
    eventMap.put("IntakePositionAgain", arm.shoulderPositionCommand(0).alongWith(intake.getIntakePivotCommand(98)).andThen(intake.getIntakePivotCommand(98.0).andThen(arm.elbowPositionCommand(20.0)).andThen(arm.shoulderPositionCommand(20).andThen(arm.elbowPositionCommand(11.75)))));
    eventMap.put("IntakeAgain", bident.grabCubeCommand(0.75).alongWith(intake.getIntakeCommand()));
    eventMap.put("ScoreMove", bident.ejectWhileOpeningCommand(() -> -.4).withTimeout(.2));
    eventMap.put("ScoreAgain", arm.elbowPositionCommand(20).andThen(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-78).until((()->arm.getElbowPostition() < 8))).andThen(intake.getIntakePivotCommand(0.0))));
    eventMap.put("ScoreMoveAgain", bident.ejectWhileOpeningCommand(() -> -.4).withTimeout(.2));

    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup("LoadingStation2P", 5, 2.25);


    FollowPathWithEvents firstPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(0), DriverStation.getAlliance()), true, drivetrain, false), paths.get(0).getMarkers(), eventMap);

    return Commands.sequence(arm.resetEncoder(), bident.autoGrabCommand(()->0.1).withTimeout(0.05).raceWith(intake.getIntakePivotCommand(95)), arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(0.1), bident.openCommand().raceWith(arm.shoulderPositionCommand(0.0)),
        firstPath);
  }

  public static Command getLoadingStationBalanceCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", intake.getIntakePivotCommand(98.0).andThen(arm.elbowPositionCommand(18.0).andThen(arm.shoulderPositionCommand(20).andThen(arm.elbowPositionCommand(11.75).alongWith(bident.grabCubeCommand(.75).alongWith(intake.getIntakeCommand()))))));
    eventMap.put("ScorePosition", arm.elbowPositionCommand(20).andThen(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-102).until((() -> arm.getElbowPostition() < 8))).andThen(intake.getIntakePivotCommand(0.0)).andThen(arm.shoulderPositionCommand(-17))));
    eventMap.put("ScoreMove", bident.ejectWhileOpeningCommand(() -> -.4).withTimeout(.2));
    eventMap.put("Retract", arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(0)).andThen(intake.getIntakePivotCommand(0.0)));

    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup("LoadingStation2B", 5, 2.25);


    FollowPathWithEvents firstPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(0), DriverStation.getAlliance()), true, drivetrain, false), paths.get(0).getMarkers(), eventMap);

    return Commands.sequence(arm.resetEncoder(), bident.autoGrabCommand(()->0.1).withTimeout(0.05).raceWith(intake.getIntakePivotCommand(95)), arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(0.1), bident.openCommand().raceWith(arm.shoulderPositionCommand(0.0)),
        firstPath,
        drivetrain.brakeCommand());
  }

  public static Command getBumpPart1Command(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", intake.getIntakePivotCommand(98.0).andThen(arm.elbowPositionCommand(18.0).andThen(arm.shoulderPositionCommand(20).andThen(arm.elbowPositionCommand(11.75).alongWith(bident.grabCubeCommand(.75).alongWith(intake.getIntakeCommand()))))));    
    eventMap.put("ScorePosition", arm.elbowPositionCommand(20).andThen(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-102).until((() -> arm.getElbowPostition() < 8))).andThen(intake.getIntakePivotCommand(0.0)).andThen(arm.shoulderPositionCommand(-17))));
    eventMap.put("IntakeAgain", arm.shoulderPositionCommand(0).alongWith(intake.getIntakePivotCommand(98)).andThen(arm.elbowPositionCommand(18.0).andThen(arm.shoulderPositionCommand(20).andThen(arm.elbowPositionCommand(11.75).alongWith(bident.grabCubeCommand(0.75).alongWith(intake.getIntakeCommand()))))));
    //eventMap.put("Retract", arm.shoulderPositionCommand(0).alongWith(intake.getIntakePivotCommand(98)).andThen(arm.elbowPositionCommand(15.0).andThen(arm.shoulderPositionCommand(20).andThen(arm.elbowPositionCommand(11.75).alongWith(bident.grabCubeCommand(0.75).alongWith(intake.getIntakeCommand()))))));
    eventMap.put("ScoreAgain", arm.elbowPositionCommand(20).andThen(arm.shoulderPositionCommand(0).andThen(arm.elbowPositionCommand(-78).until((()->arm.getElbowPostition() < 8))).andThen(intake.getIntakePivotCommand(0.0))));
    eventMap.put("ScoreMove", bident.ejectWhileOpeningCommand(() -> -.2).withTimeout(.2));
    eventMap.put("ScoreMoveAgain", bident.ejectWhileOpeningCommand(() -> -.2).withTimeout(.2));


    List<PathPlannerTrajectory> paths = PathPlanner.loadPathGroup("Bump2P", 2.5, 4);


    FollowPathWithEvents firstPath = new FollowPathWithEvents(new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
      paths.get(0), DriverStation.getAlliance()), true, drivetrain, true), paths.get(0).getMarkers(), eventMap);

    return Commands.sequence(arm.resetEncoder(), bident.autoGrabCommand(()->0.1).withTimeout(0.05).raceWith(intake.getIntakePivotCommand(95)), arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(0.1), bident.openCommand().raceWith(arm.shoulderPositionCommand(0.0)),
        firstPath);
  }

  public static Command getBalanceCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)),
        new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath("Balance", 3, 1), DriverStation.getAlliance()), true, drivetrain, false));
  }

  public static Command getBumpCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)),
        new FollowPathPlannerPath(PathPlannerTrajectory.transformTrajectoryForAlliance(
            PathPlanner.loadPath("Bump", 3, 1), DriverStation.getAlliance()), true, drivetrain, false), drivetrain.brakeCommand());
  }

  public static Command getStupidCommand(Arm arm, JoeBident bident, Drivetrain drivetrain, Intake intake) {
    return Commands.sequence(arm.elbowPositionCommand(-130.0).andThen(arm.shoulderPositionCommand(-37.0)),
        bident.openCommand().withTimeout(.5), arm.shoulderPositionCommand(0.0).andThen(arm.elbowPositionCommand(0.0)));
  }
}
