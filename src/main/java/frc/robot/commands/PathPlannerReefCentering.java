// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that centers the robot with the reef. */
public class PathPlannerReefCentering extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_drive;
  private final ElevatorSubsystem m_elevator;

  private Side lineUp;
  private Pose2d nearestReefSide;
  private Pose2d scoringPosition;
  private Setpoint setpoint;
  private boolean atHeight;
  private Command currentPathCommand;

  public enum Side {
    Left,
    Middle,
    Right,
    Back
  }
  
  public PathPlannerReefCentering(SwerveSubsystem drive, ElevatorSubsystem elevator, Side side) {
    m_drive = drive;
    m_elevator = elevator;
    lineUp = side;

    addRequirements(drive, elevator);
  }

  private Command calculatePath() {
    atHeight = m_elevator.atHeight();
    setpoint = m_elevator.getSetpoint();

    double x = nearestReefSide.getX();
    double y = nearestReefSide.getY();
    double rot = nearestReefSide.getRotation().getDegrees();

    if (atHeight)
      switch (setpoint) {
        case kLevel2:
          x += FieldPoses.L2ScoringOffset * Math.cos(rot);
          y += FieldPoses.L2ScoringOffset * Math.sin(rot);
          break;
        case kLevel3:
          x += FieldPoses.L3ScoringOffset * Math.cos(rot);
          y += FieldPoses.L3ScoringOffset * Math.sin(rot);
          break;
        case kLevel4:
          x += FieldPoses.L4ScoringOffset * Math.cos(rot);
          y += FieldPoses.L4ScoringOffset * Math.sin(rot);
          break;
        case kTopAlgae:
          x += FieldPoses.algaeScoringOffset * Math.cos(rot);
          y += FieldPoses.algaeScoringOffset * Math.sin(rot);
          break;
        case kBottomAlgae:
          x += FieldPoses.algaeScoringOffset * Math.cos(rot);
          y += FieldPoses.algaeScoringOffset * Math.sin(rot);
          break;
      }

      switch (lineUp) {
        case Left:
          x -= FieldPoses.leftOffset * Math.sin(rot);
          y += FieldPoses.leftOffset * Math.cos(rot);
          break;
        case Right:
          x += FieldPoses.leftOffset * Math.sin(rot);
          y -= FieldPoses.leftOffset * Math.cos(rot);
          break;
        case Back:
          rot += 180;
          break;
      }

    scoringPosition = new Pose2d(x, y, new Rotation2d(Math.toRadians(rot)));
    return m_drive.driveToPose(scoringPosition, PathPlannerConstants.testingConstraints, 0);
  }

  // Pseudo Code
  //
  // Find nearest reef side pose (red and blue)
  // Go to the pose plus offset for right/left
  // Wait until elevator is atHeight
  // Move forward a distance based on setpoint
  //
  // Cancel command if POV down is pressed
  // Wait until coral is scored then end?



  @Override
  public void initialize() {
    if (m_drive.isRedSide())
      nearestReefSide = m_drive.getPose().nearest(FieldPoses.redReefPoses);
    else 
      nearestReefSide = m_drive.getPose().nearest(FieldPoses.blueReefPoses);

    atHeight = m_elevator.atHeight();
    setpoint = m_elevator.getSetpoint();

    currentPathCommand = calculatePath();
    currentPathCommand.schedule();
  }

  @Override
  public void execute() {
    if (setpoint != m_elevator.getSetpoint() || atHeight != m_elevator.atHeight()) {
      currentPathCommand.cancel();
      calculatePath();
      currentPathCommand.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    currentPathCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
